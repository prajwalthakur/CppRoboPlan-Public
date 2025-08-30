#include <iostream>
#include <vector>
#include <thread>
#include "/root/workspace/src/planner/planner.h"
#include "/root/workspace/src/visualizer/visualizer.h"
#include "/root/workspace/src/models/models.h"
namespace pin = pinocchio;
int main()

{
    std::string urdfPath = "/root/workspace/src/models/panda_description/urdf/panda.urdf" ;
    std::string modelPath = "/root/workspace/src/models" ;
    std::string srdfFileName = "/root/workspace/src/models/panda_description/srdf/panda.srdf";

    //Build the kinematic model first
    pin::Model model;
    pin::urdf::buildModel(urdfPath, model);
    // create Data for the built model
    pin::Data data(model);
    
    //Build geometry models
    pin::GeometryModel visualModel{}, collisionModel{};
    std::vector<std::string> package_dirs{modelPath};
    pin::urdf::buildGeom(model, urdfPath, pin::VISUAL,    visualModel,    package_dirs);
    pin::urdf::buildGeom(model, urdfPath, pin::COLLISION, collisionModel, package_dirs);
    

    // Add & prune collision pairs 
    scenario::panda::addSelfCollisions(model,collisionModel,srdfFileName);

    // add collisions to the custom added collision objects to the links of the manipulator
    double inflationRadius = 0.0;
    scenario::panda::addObjectCollisions(model,collisionModel,visualModel,inflationRadius);

    // Build GeometryData AFTER pairs are finalized
    pin::GeometryData collisionData(collisionModel);
    
    // visualization
    cpproboplan::visualizer::VisualizerOptions visualizationOptions{};
    cpproboplan::visualizer::visJointTrajectory mVis = cpproboplan::visualizer::visJointTrajectory(model,visualModel,collisionModel,visualizationOptions);

    // create the rrt planning configuration options
    auto plannerOptions = cpproboplan::planner::RRTPlannerOptions{};
    plannerOptions.max_connection_dist = 0.9;




    // generate the #dim seed vector for generating the random Joint angles 
    std::vector<std::size_t> seedVector = cpproboplan::generateRandomSeed(plannerOptions.rng_seed, model.lowerPositionLimit.size());
    
    // create the random vector generator
    cpproboplan::crRandVecGenerator<double> randomVecGenerator = \
    cpproboplan::crCreateRandVecGenerator(plannerOptions.distribution_type,seedVector,model, plannerOptions.joint_limit_padding);

    // create the RRT planner object
    cpproboplan::planner::plSpaceType spaceType = cpproboplan::planner::plSpaceType::S_JOINTSPACE;
    cpproboplan::planner::RRT RRTPlanner = cpproboplan::planner::RRT(spaceType, model, collisionModel, plannerOptions);

    std::this_thread::sleep_for(std::chrono::duration<double>(10));
    // sample a start and goal joint pose which is not in collision
    auto [qStartPose,isSamplingSuccess1] = cpproboplan::crGetRandomCollisionFreeSample(model, 
                                                                            data, 
                                                                            collisionModel, 
                                                                            collisionData, 
                                                                            plannerOptions.collision_safety_margin ,
                                                                            plannerOptions.max_tries,
                                                                            randomVecGenerator);
    // set the starting pose in viewer

    mVis.setStartJointPose(qStartPose);

    auto [qGoalPose,isSamplingSuccess2] = cpproboplan::crGetRandomCollisionFreeSample(model, 
                                                                        data, 
                                                                        collisionModel, 
                                                                        collisionData, 
                                                                        plannerOptions.collision_safety_margin ,
                                                                        plannerOptions.max_tries,
                                                                        randomVecGenerator);
    if(!isSamplingSuccess1 || !isSamplingSuccess2)
    {
        std::cerr<<" couldnt generate a collision free samples"<<std::endl;
    }
    else
    {
        std::cerr<<" Generated Collision Free Samples "<<std::endl;

    }

    std::vector<double> startPose(qStartPose.data(),qStartPose.data()+qStartPose.size());
    std::vector<double> goalPose(qGoalPose.data(),qGoalPose.data()+qGoalPose.size());
    
    std::this_thread::sleep_for(std::chrono::duration<double>(10));

    // sove the path planning problem
    bool isSolved = RRTPlanner.solve(startPose, goalPose);

    if(!isSolved)
    {
        std::cerr<<" couldnt solve the path planning problem"<<std::endl;

    }
    else
    {
 
        std::cerr<<" solved the path planning problem!"<<std::endl;

    }
    

    auto result = RRTPlanner.getResult();
    std::vector<std::vector<double>>  solPath = result.path;
    bool isFoundPath = result.isSuccess;
    double cost = result.cost;
    
    std::cerr<< " Path size :"<< solPath.size() << std::endl;
    std::cerr<<"total Euclid distance in Joint Space: "<< cost << std::endl;


    
    
    //mVis.showStartAndGoalEEPose(qStartPose,qGoalPose);
    //std::this_thread::sleep_for(std::chrono::duration<double>(10));
    std::shared_ptr<Meshcat> meshcatPointer = mVis.getMeshcatPtr();
    meshcatPointer->StartRecording();
    meshcatPointer->PublishRecording();
    for(int i=0;i<solPath.size();++i)
    {
        mVis.stepSim(Eigen::Map<pin::Model::ConfigVectorType>(solPath[i].data(),solPath[i].size()) );
        std::cerr<<" iteration "<< i << std::endl;
    }
    meshcatPointer->StopRecording();

    //meshcatPointer->PublishRecording();
    std::this_thread::sleep_for(std::chrono::duration<double>(20));
    // std::string html = meshcatPointer->get_recording();

    // // Write to file
    // std::ofstream out("/root/workspace/src/examples/RRTPlanner/exmple");
    // out << html;
    // out.close();
    return 0;
}
