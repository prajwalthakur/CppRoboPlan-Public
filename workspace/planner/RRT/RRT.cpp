/**
 * @file RRT.cpp
 * @author Prajwal Thakur
 * @brief  source file for RRT planner.
 *
 * */
#include "RRT.hpp"
namespace pin = pinocchio;

namespace cpproboplan::planner
{
    /**
     * @brief Default constructor for the RRT planner.
     */
    RRT::RRT()=default;

    /**
     * @brief Default destructor for the RRT planner.
     */
    RRT::~RRT()=default;
    
    /**
     * @brief Parameterized constructor for the RRT planner.
     * @param spaceType The type of planning space (joint or Euclidean).
     * @param model Pinocchio robot model.
     * @param collisionModel Pinocchio robot collision model.
     * @param plannerOptions Configuration options for the RRT algorithm.
     */
    RRT::RRT(const plSpaceType& spaceType, 
    const pin::Model& model,
    const pin::GeometryModel& collisionModel, 
    const RRTPlannerOptions& plannerOptions):
    mPlannerOptions(plannerOptions), 
    mPinModel(model), 
    mSpaceType(spaceType), 
    mData(mPinModel),
    mCollisionModel(collisionModel),
    mCollisionData(mCollisionModel),
    PlannerBase(model.lowerPositionLimit.size()),
    mKdTree(std::make_unique<kdTreeType>(getDim()))
    {
        init();
    }
    
    /**
     * @brief Initializes the planner.
     * * This function sets up the random number generators, the state space, and the
     * KD-Tree for the RRT algorithm based on the provided planner options.
     */
    void RRT::init()
    {
        std::vector<std::size_t> seedVector = cpproboplan::generateRandomSeed(mPlannerOptions.rng_seed, getDim());
        
        mRandomVecGenerator = cpproboplan::crCreateRandVecGenerator(mPlannerOptions.distribution_type, 
        seedVector,mPinModel, mPlannerOptions.joint_limit_padding);

        // create a uniform random number  generator to sample the goal 
        std::vector<std::size_t> seedForGoalBiasing = cpproboplan::generateRandomSeed(mPlannerOptions.rng_seed, 1);
        mRandomNumGenerator = cpproboplan::crCreateRandomGenerator(mPlannerOptions.distribution_type,seedForGoalBiasing[0],0.0,1.0);

        // Sample Space for RRT (Option: Joint Space or Euclidean Space)
        if (mSpaceType == plSpaceType::S_JOINTSPACE) 
        {
            mPlStateSpacePtr = std::make_shared<plStateSpace<plJointState, double>>(getDim());
        } 
        mKdTree->setRebalanceRatioThreshold(2.0);
    }

    /**
     * @brief Solves the planning problem from a start to a goal pose.
     * * The main RRT algorithm loop. It iteratively samples the state space, finds the nearest
     * node in the tree, extends a new node towards the sample, checks for collisions, and adds the
     * new node to the tree. The loop terminates when a path is found or the maximum time is reached.
     * @param startPose A vector of doubles representing the starting configuration.
     * @param goalPose A vector of doubles representing the goal configuration.
     * @return true If a path is found.
     * @return false If a path is not found within the given constraints.
     */
    bool RRT::solve(std::vector<double>& startPose, std::vector<double>& goalPose)
    {
        auto node     =     std::make_shared<plNodeType>(mPlStateSpacePtr, startPose);
        auto goalNode =     std::make_shared<plNodeType>(mPlStateSpacePtr, goalPose);
        double dist = 0.0;
        mKdTree->add(node);
        auto st = node->getState();
        auto gl = goalNode->getState();
        dist = cpproboplan::distancemetric::Euclidean::calcDistance(st,gl);
        bool isPathFound = false;
        bool isSubPathCollisionFree = false;
        plSharedNodePtr  finalNode = nullptr;
        if(dist <=  mPlannerOptions.max_connection_dist)
        {
            isSubPathCollisionFree = discretizeAndCheckCollision(node,goalNode,mPinModel,
                                    mCollisionModel,
                                    mData,
                                    mCollisionData,
                                    mPlannerOptions.max_step_size,
                                    mPlannerOptions.collision_safety_margin,
                                    true);
            if(isSubPathCollisionFree)
            {
                isPathFound = true;
                goalNode->setParent(node.get());
                finalNode = goalNode;
                
            }
        }

        std::pair<pin::Model::ConfigVectorType,bool> sampledResult;
        pin::Model::ConfigVectorType qRandom;
        bool isSamplingSuccess = false;
        int  i = 0;
        setStartTime(std::chrono::high_resolution_clock::now());
        auto duration = getElapsedTime(std::chrono::high_resolution_clock::now());
        while(!isPathFound &&  duration <=10)
        {
            ++i;
            if(mRandomNumGenerator.getRandomNumber() <= mPlannerOptions.goal_biasing_probability)
            {
                qRandom = Eigen::Map<pin::Model::ConfigVectorType>(gl.data(), gl.size());
                isSamplingSuccess = true;
            }
            else
            {
                sampledResult = cpproboplan::crGetRandomCollisionFreeSample(mPinModel, 
                                                                            mData, 
                                                                            mCollisionModel, 
                                                                            mCollisionData, 
                                                                            mPlannerOptions.collision_safety_margin ,
                                                                            mPlannerOptions.max_tries,
                                                                            mRandomVecGenerator);
                qRandom = sampledResult.first;
                isSamplingSuccess  = sampledResult.second;
            }

            if(!isSamplingSuccess){continue;}
            node.reset();
            std::vector<double> stdVec(qRandom.data(),qRandom.data() + qRandom.size());
            node = std::make_shared<plNodeType>(mPlStateSpacePtr, stdVec);
            auto parentNode = mKdTree->searchNN(node);
            
            // extend function 
            generateSteerNode(parentNode, node, mPlannerOptions.max_connection_dist);

            isSubPathCollisionFree = discretizeAndCheckCollision(parentNode,node,mPinModel,
                                    mCollisionModel,
                                    mData,
                                    mCollisionData,
                                    mPlannerOptions.max_step_size,
                                    mPlannerOptions.collision_safety_margin,
                                    true);
            if(!isSubPathCollisionFree){continue;}

            node->setParent(parentNode.get());
            mKdTree->add(node);
            dist = cpproboplan::distancemetric::Euclidean::calcDistance(goalNode->getState(),node->getState());
            if(dist <= mPlannerOptions.max_connection_dist)
            {
                isPathFound = true;
                goalNode->setParent(node.get());
                finalNode = goalNode;
            }
            duration = getElapsedTime(std::chrono::high_resolution_clock::now());
        }

        if(!isPathFound)
        {
            std::cerr<<"Path Not Found " << "time elapsed: "<< duration << " final euclid-distance to goal joint configuration:"<<dist<<std::endl;
            finalNode = node;
        }
        else if(isPathFound)
        {
            std::cerr<<"Path Found " << "time elapsed: "<< duration << " final euclid-distance to goal joint configuration:"<<dist<<std::endl;
        }
        constructResult(finalNode.get(), isPathFound);

        return isPathFound;
    }
}