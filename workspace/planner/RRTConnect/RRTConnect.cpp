/**
 * @file RRTConnect.cpp
 * @author Prajwal Thakur
 * @brief  source file for RRT-Connect planner.
 *
 * */
#include "RRTConnect.hpp"
namespace pin = pinocchio;

namespace cpproboplan::planner
{
    /**
     * @brief Default constructor for the RRTConnect planner.
     */
    RRTConnect::RRTConnect()=default;

    /**
     * @brief Default destructor for the RRTConnect planner.
     */
    RRTConnect::~RRTConnect()=default;
    
    /**
     * @brief Parameterized constructor for the RRTConnect planner.
     * @param spaceType The type of planning space (joint or Euclidean).
     * @param model Pinocchio robot model.
     * @param collisionModel Pinocchio robot collision model.
     * @param plannerOptions Configuration options for the RRT-Connect algorithm.
     */
    RRTConnect::RRTConnect(const plSpaceType& spaceType, 
    const pin::Model& model,
    const pin::GeometryModel& collisionModel, 
    const RRTConPlannerOptions& plannerOptions):
    mPlannerOptions(plannerOptions), 
    mPinModel(model), 
    mSpaceType(spaceType), 
    mData(mPinModel),
    mCollisionModel(collisionModel),
    mCollisionData(mCollisionModel),
    PlannerBase(model.lowerPositionLimit.size()),
    mStartPhaseKdTree(std::make_shared<kdTreeType>(getDim())),
    mGoalPhaseKdTree(std::make_shared<kdTreeType>(getDim()))
    {
        init();
    }
    
    /**
     * @brief Initializes the planner.
     * * This function sets up the random number generators, state space, and the two KD-Trees
     * for the start and goal trees based on the provided planner options.
     */
    void RRTConnect::init()
    {
        
        std::vector<std::size_t> seedVector = cpproboplan::generateRandomSeed(mPlannerOptions.rng_seed, getDim());
        
        mRandomVecGenerator = cpproboplan::crCreateRandVecGenerator(mPlannerOptions.distribution_type, 
        seedVector,mPinModel, mPlannerOptions.joint_limit_padding);

        // create a uniform random number  generator to sample the goal 
        std::vector<std::size_t> seedForGoalBiasing = cpproboplan::generateRandomSeed(mPlannerOptions.rng_seed, 1);
        mRandomNumGenerator = cpproboplan::crCreateRandomGenerator(mPlannerOptions.distribution_type,seedForGoalBiasing[0],0.0,1.0);

        //sample Space for RRTConnect (Option: Joint Space or Euclidean Space)
        
        if (mSpaceType == plSpaceType::S_JOINTSPACE) 
        {
            mPlStateSpacePtr = std::make_shared<plStateSpace<plJointState, double>>(getDim());
        }

        mStartPhaseKdTree->setRebalanceRatioThreshold(1.5);
        mGoalPhaseKdTree->setRebalanceRatioThreshold(1.5);
    }

    //-----------------------------------------------------

    /**
     * @brief Solves the planning problem from a start to a goal pose.
     * * This is the main RRT-Connect algorithm. It grows two trees (one from the start, one from the goal)
     * by alternately extending each tree towards random samples. It also attempts to connect the two trees
     * during each iteration. The process stops when a connection is successfully made or a time limit is reached.
     * @param startPose A vector of doubles representing the starting configuration.
     * @param goalPose A vector of doubles representing the goal configuration.
     * @return true If a path is found.
     * @return false If a path is not found within the given constraints.
     */
    bool RRTConnect::solve(std::vector<double>& startPose, std::vector<double>& goalPose)
    {
        auto node     =     std::make_shared<plNodeType>(mPlStateSpacePtr, startPose);
        auto goalNode =     std::make_shared<plNodeType>(mPlStateSpacePtr, goalPose);
        double dist = 0.0;
        mStartPhaseKdTree->add(node);
        mGoalPhaseKdTree->add(node);
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
        

        // first iteration is from the start tree
        mCurrKdTreePtr = mStartPhaseKdTree;
        mOtherKdTreePtr = mGoalPhaseKdTree;
        std::string currentPhase = "start";
        std::shared_ptr<plNodeType> parentNode{nullptr};
        while(!isPathFound &&  duration <=10)
        {
            mCurrKdTreePtr = (currentPhase=="start")? mStartPhaseKdTree: mGoalPhaseKdTree;
            mOtherKdTreePtr = (currentPhase=="start")? mGoalPhaseKdTree: mStartPhaseKdTree;
            
            
            if(mRandomNumGenerator.getRandomNumber() <= mPlannerOptions.goal_biasing_probability)
            {
                // if the current phase iteration is start then goalNode would the actual goal pose otherwise it's start Pose
                qRandom = (currentPhase=="start")?Eigen::Map<pin::Model::ConfigVectorType>(gl.data(), gl.size()):
                                                Eigen::Map<pin::Model::ConfigVectorType>(st.data(), st.size());
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

            // delete the previously created node
            node.reset();
            std::vector<double> stdVec(qRandom.data(),qRandom.data() + qRandom.size());
            node = std::make_shared<plNodeType>(mPlStateSpacePtr, stdVec);
            
            // extend function
            parentNode = mCurrKdTreePtr->searchNN(node) ; 
            isSubPathCollisionFree = extendNode(parentNode,node); // extend the node from parentNode to node and modify the node itself
            if(!isSubPathCollisionFree){continue;}
            node->setParent(parentNode.get());
            mCurrKdTreePtr->add(node);
            auto stateVecCurrTree = node->getState();

            // connect step
            parentNode = mOtherKdTreePtr->searchNN(node); // node is modifed to newNode which is a attached to currKdTree in extend function
            isSubPathCollisionFree = ConnectNode(parentNode,node); // extend the node from parentNode to node; its modify the node in place 
            if(!isSubPathCollisionFree){continue;}            
            node->setParent(parentNode.get());
            mOtherKdTreePtr->add(node);
            auto stateVecOtherTree = node->getState();
            

            // check if the latest node from both the trees are within the threshold , if yes then terminate the plaanning
            dist = cpproboplan::distancemetric::Euclidean::calcDistance(stateVecCurrTree, stateVecOtherTree);
            if(dist <= mPlannerOptions.max_connection_dist)
            {
                isPathFound = true;
                goalNode->setParent(node.get());
                finalNode = goalNode;
            }


            // change the current phase for the next iteration
            currentPhase = (currentPhase=="start")? "goal": "start";
            ++i;
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

        if(mPlannerOptions.postProcess)
        {
            postProcess(mPlannerOptions.max_connection_dist);
        }

        return isPathFound;
    }

    //-----------------------------------------------------

    /**
     * @brief Extends a node towards a given target.
     * * This is a key step in the RRT-Connect algorithm. It extends the `parentNode` towards the `node`
     * by a single step, ensuring the path segment is collision-free. The `node` object is modified
     * in place to hold the new state.
     * @param parentNode The node from which to extend.
     * @param node The node representing the target configuration.
     * @return true if the extended segment is collision-free, false otherwise.
     */
    bool RRTConnect::extendNode(plSharedNodePtr parentNode, plSharedNodePtr node)
    {
        generateSteerNode(parentNode, node, mPlannerOptions.max_connection_dist);
        bool isSubPathCollisionFree = discretizeAndCheckCollision(parentNode,node,mPinModel,
                    mCollisionModel,
                    mData,
                    mCollisionData,
                    mPlannerOptions.max_step_size,
                    mPlannerOptions.collision_safety_margin,
                    true);
        return isSubPathCollisionFree;
    }
    
    //-----------------------------------------------------

    /**
     * @brief Attempts to connect a node to the other tree.
     * * This function attempts to extend the `parentNode` (from one tree) repeatedly towards the `node`
     * (from the other tree) in a greedy fashion until either a connection is made or a collision is
     * detected. This is what gives RRT-Connect its efficiency.
     * @param parentNode The nearest node in the other tree.
     * @param node The node from the current tree.
     * @return true if a successful connection is made, false otherwise.
     */
    bool RRTConnect::ConnectNode(plSharedNodePtr parentNode, plSharedNodePtr node)
    {   
        Eigen::Map<pin::Model::ConfigVectorType> qStart(parentNode->getStateRef().data(), parentNode->getStateRef().size());
        Eigen::Map<pin::Model::ConfigVectorType> qEnd(node->getStateRef().data() , node->getStateRef().size());
        double dist =  (qEnd-qStart).norm();        
        bool isSubPathCollisionFree = true;
        while ( dist > mPlannerOptions.max_connection_dist  && isSubPathCollisionFree )
        {
            generateSteerNode(parentNode, node, mPlannerOptions.max_connection_dist);
            isSubPathCollisionFree = discretizeAndCheckCollision(parentNode,node,mPinModel,
                        mCollisionModel,
                        mData,
                        mCollisionData,
                        mPlannerOptions.max_step_size,
                        mPlannerOptions.collision_safety_margin,
                        true);
            dist =  (qEnd-qStart).norm();
        }

        return isSubPathCollisionFree;
    }
}