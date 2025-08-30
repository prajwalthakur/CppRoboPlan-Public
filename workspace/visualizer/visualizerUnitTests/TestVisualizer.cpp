
#include <drake/geometry/meshcat.h>
#include <drake/geometry/shape_specification.h>   // Mesh, Box, etc.
#include <drake/math/rigid_transform.h>
#include "/root/workspace/src/core/core.h"
#include "/root/workspace/src/core/coreTest/TestFixture.hpp"
#include "/root/workspace/src/planner/base/KdTree.hpp"
#include "/root/workspace/src/planner/base/Node.h"
#include "/root/workspace/src/planner/base/StateSpaceInformation.h"
#include "/root/workspace/src/planner/base/StateInformation.h"
#include "/root/workspace/src/core/Memory.hpp"
#include "/root/workspace/src/planner/RRT/RRT.hpp"
#include "/root/workspace/src/planner/RRT/RRTPlannerOptions.h"
#include "/root/workspace/src/visualizer/VisJointTrajectory.hpp"

using namespace cpproboplan::visualizer;

namespace pin = pinocchio;
using drake::geometry::Meshcat;
using drake::geometry::Mesh;
using drake::math::RigidTransformd;
std::string urdfPath = "/root/workspace/src/models/panda_description/urdf/panda.urdf" ;
std::string modelPath = "/root/workspace/src/models" ;

// using plNodeType            =          plNode<plJointState, double>;
// using plSharedNodePtr     = cpproboplan::rplSharedPtr<plNode<plJointState, double>>;

// using plSharedConstNodePtr = cpproboplan::rplSharedPtr<const plNode<plJointState, double>>;
// using plSharedStateSpacePtrF = cpproboplan::rplSharedPtr<plStateSpace<plJointState, double>>;

// using kdTreeType = plKdTree<plJointState,double>;
// using kdTreeUniquePtr        =  cpproboplan::rplUniquePtr<kdTreeType>;


// Test fixture for kdTree
// Test fixture for kdTree
class TestVisualizer : public rplTestFixture
{
public:
    TestVisualizer()
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
        mPlannerOptions = cpproboplan::planner::RRTPlannerOptions{};

        std::vector<std::size_t> seedVector = cpproboplan::generateRandomSeed(mPlannerOptions.rng_seed, model.lowerPositionLimit.size());
        
        mRandomVecGenerator = cpproboplan::crCreateRandVecGenerator(mPlannerOptions.distribution_type, 
        seedVector,model, mPlannerOptions.joint_limit_padding);
        
    }

protected:
    cpproboplan::planner::plSpaceType spaceType = cpproboplan::planner::plSpaceType::S_JOINTSPACE;
    cpproboplan::planner::RRTPlannerOptions mPlannerOptions{};
    cpproboplan::visualizer::VisualizerOptions mVisualizerOptions{};
    pin::Model model{};
    pin::Data data;                 // create after model is ready
    pin::GeometryData mCollisionData; 
    pin::GeometryModel visual_model{}, collision_model{};
    cpproboplan::crRandVecGenerator<double> mRandomVecGenerator;
};



// ////////////////////////////////////////////////////////////////////////////////
// Test No: 1 - check creation of visualization CLASS
// rplTest(TestVisualizer, TestVisualizerClassCreation)
// {   
    
//     cpproboplan::visualizer::visJointTrajectory mVis = cpproboplan::visualizer::visJointTrajectory(model,visual_model,collision_model,mVisualizerOptions);
    
//     EXPECT_NE(mVis.getMeshcatPtr(),nullptr);
// }


// ////////////////////////////////////////////////////////////////
// Test No 1 - check path finding function when start and goal node is same
rplTest(TestVisualizer, TestVisualizerStartAndEndPoseVis)
{   
    //cpproboplan::planner::RRT RRTPlanner = RRT(spaceType, model, collision_model, mPlannerOptions);
    // cpproboplan::visualizer::visJointTrajectory mVis = cpproboplan::visualizer::visJointTrajectory(model,visual_model,collision_model,mVisualizerOptions);
    
    // auto [qRandom1,isSamplingSuccess1] = cpproboplan::crGetRandomCollisionFreeSample(model, 
    //                                                                         data, 
    //                                                                         collision_model, 
    //                                                                         mCollisionData, 
    //                                                                         mPlannerOptions.collision_safety_margin ,
    //                                                                         mPlannerOptions.max_tries,
    //                                                                         mRandomVecGenerator);
    
    // auto [qRandom2,isSamplingSuccess2] = cpproboplan::crGetRandomCollisionFreeSample(model, 
    //                                                                         data, 
    //                                                                         collision_model, 
    //                                                                         mCollisionData, 
    //                                                                         mPlannerOptions.collision_safety_margin ,
    //                                                                         mPlannerOptions.max_tries,
    //                                                                         mRandomVecGenerator);  

    //mVis.showStartAndGoalEEPose(qRandom1,qRandom2);
    //std::this_thread::sleep_for(std::chrono::duration<double>(20));

}
