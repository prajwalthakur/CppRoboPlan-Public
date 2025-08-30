
#include "/root/workspace/src/core/core.h"
#include "/root/workspace/src/core/coreTest/TestFixture.hpp"
#include "/root/workspace/src/planner/base/KdTree.hpp"
#include "/root/workspace/src/planner/base/Node.h"
#include "/root/workspace/src/planner/base/StateSpaceInformation.h"
#include "/root/workspace/src/planner/base/StateInformation.h"
#include "/root/workspace/src/core/Memory.hpp"
#include "/root/workspace/src/planner/RRT/RRT.hpp"
#include "/root/workspace/src/planner/RRT/RRTPlannerOptions.h"
#include <drake/geometry/meshcat.h>
#include <drake/geometry/shape_specification.h>   // Mesh, Box, etc.
#include <drake/math/rigid_transform.h>

using namespace cpproboplan::planner;

namespace pin = pinocchio;
using drake::geometry::Meshcat;
using drake::geometry::Mesh;
using drake::math::RigidTransformd;
std::string urdfPath = "/root/workspace/src/models/panda_description/urdf/panda.urdf" ;
std::string modelPath = "/root/workspace/src/models" ;

using plNodeType            =          plNode<plJointState, double>;
using plSharedNodePtr     = cpproboplan::rplSharedPtr<plNode<plJointState, double>>;

using plSharedConstNodePtr = cpproboplan::rplSharedPtr<const plNode<plJointState, double>>;
using plSharedStateSpacePtrF = cpproboplan::rplSharedPtr<plStateSpace<plJointState, double>>;

using kdTreeType = plKdTree<plJointState,double>;
using kdTreeUniquePtr        =  cpproboplan::rplUniquePtr<kdTreeType>;


// Test fixture for kdTree
// Test fixture for kdTree
class TestRRTPlanner : public rplTestFixture
{
public:
    TestRRTPlanner()
    {
        // 1) Build the model into the MEMBER, not a local
        pin::urdf::buildModel(urdfPath, model);

        // 2) Create Data AFTER model is built
        data = pin::Data(model);
        
        // 3) Build geometry models into MEMBERS, not locals
        std::vector<std::string> package_dirs{modelPath};
        pin::urdf::buildGeom(model, urdfPath, pin::VISUAL,    visual_model,    package_dirs);
        pin::urdf::buildGeom(model, urdfPath, pin::COLLISION, collision_model, package_dirs);
        mCollisionData = pin::GeometryData(collision_model);

        // 4) Initialize planner options (member)
        mPlannerOptions = RRTPlannerOptions{};

        std::vector<std::size_t> seedVector = cpproboplan::generateRandomSeed(mPlannerOptions.rng_seed, model.lowerPositionLimit.size());
        
        mRandomVecGenerator = cpproboplan::crCreateRandVecGenerator(mPlannerOptions.distribution_type, 
        seedVector,model, mPlannerOptions.joint_limit_padding);
        
    }

protected:
    plSpaceType spaceType = plSpaceType::S_JOINTSPACE;
    cpproboplan::planner::RRTPlannerOptions mPlannerOptions{};
    pin::Model model{};
    pin::Data data;                 // create after model is ready
    pin::GeometryData mCollisionData; 
    pin::GeometryModel visual_model{}, collision_model{};
    cpproboplan::crRandVecGenerator<double> mRandomVecGenerator;
};



// ////////////////////////////////////////////////////////////////////////////////
// Test No: 1 - check creation of RRT CLASS

rplTest(TestRRTPlanner, TestRRTClassCreation)
{   
    cpproboplan::planner::RRT RRTPlanner = RRT(spaceType, model, collision_model, mPlannerOptions);
    std::size_t dim = RRTPlanner.getDim();
    EXPECT_EQ(dim, 9) << "dim is " << dim
                      << " njoints=" << model.njoints
                      << " lowerPositionLimit.size()=" << model.lowerPositionLimit.size();
}


////////////////////////////////////////////////////////////////
// Test No 2 - check path finding function when start and goal node is same
rplTest(TestRRTPlanner, TestRRTFindPathSingleNode)
{   
    cpproboplan::planner::RRT RRTPlanner = RRT(spaceType, model, collision_model, mPlannerOptions);
    
    auto [qRandom,isSamplingSuccess] = cpproboplan::crGetRandomCollisionFreeSample(model, 
                                                                            data, 
                                                                            collision_model, 
                                                                            mCollisionData, 
                                                                            mPlannerOptions.collision_safety_margin ,
                                                                            mPlannerOptions.max_tries,
                                                                            mRandomVecGenerator);
    EXPECT_EQ(isSamplingSuccess,true);
    std::vector<double> startPose(qRandom.data(),qRandom.data()+qRandom.size());
    std::vector<double> goalPose= startPose;
    bool isSolved = RRTPlanner.solve(startPose, goalPose);
    EXPECT_EQ(isSolved,true);
}

////////////////////////////////////
// Test 3 : random sample all should be different ?
rplTest(TestRRTPlanner, TestRRTRandomSamples)
{   
    //cpproboplan::planner::RRT RRTPlanner = RRT(spaceType, model, collision_model, mPlannerOptions);
    
    auto [qRandom,isSuccess1] = cpproboplan::crGetRandomCollisionFreeSample(model, 
                                                                            data, 
                                                                            collision_model, 
                                                                            mCollisionData, 
                                                                            mPlannerOptions.collision_safety_margin ,
                                                                            mPlannerOptions.max_tries,
                                                                            mRandomVecGenerator);
    auto [qRandom2,isSuccess2] = cpproboplan::crGetRandomCollisionFreeSample(model, 
                                                                            data, 
                                                                            collision_model, 
                                                                            mCollisionData, 
                                                                            mPlannerOptions.collision_safety_margin ,
                                                                            mPlannerOptions.max_tries,
                                                                            mRandomVecGenerator);    
    EXPECT_TRUE(qRandom != qRandom2);

}


///////////////////////////////////////
// Test No 4 RRT path Finding when start and goals are different

rplTest(TestRRTPlanner, TestRRTFindPathRandomNodes)
{   
    
    
    mPlannerOptions.max_connection_dist = 0.9;
    cpproboplan::planner::RRT RRTPlanner = RRT(spaceType, model, collision_model, mPlannerOptions);
    
    auto [qRandom,isSamplingSuccess] = cpproboplan::crGetRandomCollisionFreeSample(model, 
                                                                            data, 
                                                                            collision_model, 
                                                                            mCollisionData, 
                                                                            mPlannerOptions.collision_safety_margin ,
                                                                            mPlannerOptions.max_tries,
                                                                            mRandomVecGenerator);
    EXPECT_EQ(isSamplingSuccess,true);
    
    std::vector<double> startPose(qRandom.data(),qRandom.data()+qRandom.size());
    auto [qRandom2,isSamplingSuccess2] = cpproboplan::crGetRandomCollisionFreeSample(model, 
                                                                            data, 
                                                                            collision_model, 
                                                                            mCollisionData, 
                                                                            mPlannerOptions.collision_safety_margin ,
                                                                            mPlannerOptions.max_tries,
                                                                            mRandomVecGenerator);
    //auto qRandom2 = qRandom;
    // Eigen::VectorXd qRandom2 = qRandom;   // copy
    //qRandom2.head(4).array() += 0.5;
    std::vector<double> goalPose(qRandom2.data(),qRandom2.data()+qRandom2.size());
    bool isSolved = RRTPlanner.solve(startPose, goalPose);
    EXPECT_EQ(isSolved,true);
}



// ////////////////////////////////////////////////////////////////////////////////
// // Test No: 2 - check Adding Node in KD Tree

// rplTest(TeskdTree, TestkdTreeCreation)
// {
//     kdTreeUniquePtr kdTree = std::make_unique<plKdTree<plJointState,double>>(dim);
//     EXPECT_EQ(kdTree->getDim(), dim);
//     EXPECT_EQ(kdTree->getRoot(), nullptr);
//     EXPECT_EQ(kdTree->getDepth(), 0);
    
// }

// ////////////////////////////////////////////////////////////////////////////////
// // Test No: 3 - checking Nearest Neighbour function

// rplTest(TeskdTree, TestkdTreeAddingNodes)
// {
//     kdTreeUniquePtr kdTree = std::make_unique<plKdTree<plJointState,double>>(dim);
//     kdTree->add(node1);
//     auto nn = kdTree->getRoot();
//     ASSERT_NE(nn, nullptr)<< " root is a null pointer"; // gtest assertion, avoids segfault
//     EXPECT_EQ(nn->idx,0);
//     auto node = kdTree->getPlNodeAtindexi(0);
//     EXPECT_TRUE(node==node1);
//     auto NNode = kdTree->searchNN(node1);
//     EXPECT_TRUE(NNode !=nullptr);
//     EXPECT_TRUE(NNode == node1);
//     NNode = kdTree->searchNN(node2);
//     EXPECT_TRUE(NNode==node1);
//     kdTree->add(node3);
//     NNode = kdTree->searchNN(node2);
//     EXPECT_TRUE(NNode==node3);
// }

// ////////////////////////////////////////////////////////////////////////////////
// // Test No: 4 - checking balancing

// rplTest(TeskdTree, TestkdTreeNotBalance)
// {
//     kdTreeUniquePtr kdTree = std::make_unique<plKdTree<plJointState,double>>(dim);
//     kdTree->setRebalanceRatioThreshold(2.0);
//     kdTree->add(node1);
//     EXPECT_EQ(kdTree->getDepth(), 0)<< "1. depth is : " << kdTree->getDepth();
//     kdTree->add(node3);
//     EXPECT_EQ(kdTree->getDepth(), 1)<< "2. depth is : " << kdTree->getDepth();
//     EXPECT_TRUE(kdTree->getRebalanceRatio()>=1.0)<< "2. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;;
//     kdTree->add(node2);
//     EXPECT_EQ(kdTree->getDepth(), 2)<< "3 . depth is : " << kdTree->getDepth();
//     EXPECT_TRUE(kdTree->getRebalanceRatio()<=1.0)<< "3. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;
//     kdTree->add(node4);
//     EXPECT_EQ(kdTree->getDepth(), 3)<< "4. depth is : " << kdTree->getDepth();
//     EXPECT_TRUE(kdTree->getRebalanceRatio()>=1.5)<< "4. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;
// }


// ////////////////////////////////////////////////////////////////////////////////
// // Test No: 5 - checking balancing

// rplTest(TeskdTree, TestkdTreeBalance)
// {
//     kdTreeUniquePtr kdTree = std::make_unique<plKdTree<plJointState,double>>(dim);
//     kdTree->setRebalanceRatioThreshold(1.1);
//     kdTree->add(node1);
//     EXPECT_EQ(kdTree->getDepth(), 0)<< "1. depth is : " << kdTree->getDepth();
//     kdTree->add(node3);
//     EXPECT_EQ(kdTree->getDepth(), 1)<< "2. depth is : " << kdTree->getDepth();
//     EXPECT_TRUE(kdTree->getRebalanceRatio()>=1.0)<< "2. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;;
//     kdTree->add(node2);
//     EXPECT_EQ(kdTree->getDepth(), 2)<< "3 . depth is : " << kdTree->getDepth();
//     EXPECT_TRUE(kdTree->getRebalanceRatio()<=1.0)<< "3. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;
//     kdTree->add(node4);
//     EXPECT_EQ(kdTree->getDepth(), 2)<< "4. depth is : " << kdTree->getDepth();
//     EXPECT_NEAR(kdTree->getRebalanceRatio(),1.0,1e-9)<< "4. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;
// }