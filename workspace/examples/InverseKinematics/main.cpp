/**
 * @file main.cpp
 * @brief Example program that builds a Panda robot model with Pinocchio, sets up
 *        visualization and collision geometry, samples start/goal joint states,
 *        and solves a Inverse Kinematics problem using LM method.
 *
 *
 * @copyright
 *   (c) 2025 Prajwal Thakur.
 *
 * @note The example expects the Panda URDF/SRDF and meshes under @c /root/workspace/src/models .
 * @warning Paths are absolute for convenience; consider using a resource locator in production.
 */

#include <iostream>
#include <vector>
#include <thread>

#include "/root/workspace/src/planner/planner.h"
#include "/root/workspace/src/visualizer/visualizer.h"
#include "/root/workspace/src/models/models.h"
#include "/root/workspace/src/inverseKinematics/InverseKinematics.h"

/// Shorthand alias for Pinocchio namespace.
namespace pin = pinocchio;

/**
 * @brief Entry point: builds models, configures collision/visualization, and runs RRT-Connect.
 *
 * @return 0 on success, non-zero otherwise.
 */
int main()
{
    /** @name Resource paths (URDF/SRDF and package root)
     *  Absolute paths are used here for simplicity.
     *  @{ */
    std::string urdfPath = "/root/workspace/src/models/panda_description/urdf/panda.urdf" ;  ///< Panda URDF file
    std::string modelPath = "/root/workspace/src/models" ;                                   ///< Root for mesh packages
    std::string srdfFileName = "/root/workspace/src/models/panda_description/srdf/panda.srdf";///< Panda SRDF file
    /** @} */

    /** @brief Build the kinematic model (joint topology, limits, etc.). */
    pin::Model model;
    pin::urdf::buildModel(urdfPath, model);

    /// @brief Allocate runtime data for algorithms on the built model.
    pin::Data data(model);
    
    /** @name Build geometry models for visualization and collision checking
     *  Both visual and collision geometry are loaded from URDF with package search dirs.
     *  @{ */
    pin::GeometryModel visualModel{}, collisionModel{};
    std::vector<std::string> package_dirs{modelPath};
    pin::urdf::buildGeom(model, urdfPath, pin::VISUAL,    visualModel,    package_dirs);
    pin::urdf::buildGeom(model, urdfPath, pin::COLLISION, collisionModel, package_dirs);
    /** @} */

    /**
     * @brief Add & prune self-collision pairs using SRDF rules.
     * @details This step removes kinematic adjacencies and disabled pairs to speed up checks mentioned in SRDF file.
     */
    scenario::panda::addSelfCollisions(model, collisionModel, srdfFileName);

    /**
     * @brief Attach additional collision objects to links (e.g., grippers, tools).
     * @param inflationRadius Optional padding applied to attached objects (meters).
     */
    double inflationRadius = 0.0;
    scenario::panda::addObjectCollisions(model, collisionModel, visualModel, inflationRadius);

    /// @brief Allocate geometry data AFTER collision pairs are finalized.
    pin::GeometryData collisionData(collisionModel);
    
    /** @name Visualization setup (Meshcat-based)
     *  Create a trajectory visualizer for joint-space playback.
     *  @{ */
    cpproboplan::visualizer::VisualizerOptions visualizationOptions{};
    visualizationOptions.ee_frame_name = "panda_hand";
    cpproboplan::visualizer::visJointTrajectory mVis =
        cpproboplan::visualizer::visJointTrajectory(model, visualModel, collisionModel, visualizationOptions);
    /** @} */

    /** @name Planner setup
     *  Configure RRT-Connect in joint space and random sampling utilities.
     *  @{ */
    auto plannerOptions = cpproboplan::planner::RRTConPlannerOptions{}; ///< Planner configuration

    // To reproduce runs, set a seed in plannerOptions .  or for random run set the seed equal to current time if desired.
    // plannerOptions.rng_seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::cerr << "random seed " << plannerOptions.rng_seed << std::endl;

    /// Create a per-DOF seed vector used by the random joint sampler.
    std::vector<std::size_t> seedVector =
        cpproboplan::generateRandomSeed(plannerOptions.rng_seed, model.lowerPositionLimit.size());
    
    /// Construct a random vector generator under joint limits (+ optional padding).
    cpproboplan::crRandVecGenerator<double> randomVecGenerator =
        cpproboplan::crCreateRandVecGenerator(
            plannerOptions.distribution_type, seedVector, model, plannerOptions.joint_limit_padding);


    /** @name Start/Goal state selection
     *  You can either hard-code states or sample collision-free ones.
     *  @{ */

    
    // we will try to find Joint Vector of the goal, given initial guess as qStartPose
    pin::Model::ConfigVectorType qGoalPose(9); ///< Desired goal pose in joint space
    qGoalPose << 2.01141, 1.71381, 2.55119, -2.3794, 1.92879, 2.47939, -2.20627, 0.00275953, 0.0209803;
    bool isSamplingSuccess2 = true;
    // find the SE(3) pose of the qGoalPose

    std::string targetFrame = "panda_hand";
    std::size_t targetFrameId = model.getFrameId(targetFrame);
    pin::forwardKinematics(model,data,qGoalPose);
    pin::updateFramePlacements(model, data);
    const pin::SE3 oMdes = data.oMf[targetFrameId];
    // ignored frames
    std::vector<std::string> ignoredJointNames = {"panda_finger_joint1","panda_finger_joint2"};
    std::vector<std::size_t> ignoredJointIds;
    std::cerr<<"indices of panda_joint1 " << model.getJointId("panda_joint1") << std::endl;
    for(auto frameName : ignoredJointNames)
    {
        ignoredJointIds.emplace_back(model.getJointId(frameName));
        std::cerr<<"ignored joint indices : " << model.getJointId(frameName) << std::endl;
    }
    
     
    pin::Model::ConfigVectorType qStartPose(9); ///< 7 Panda joints + 2 finger joints (example)
    // Hard-coded start pose (assumed within limits):
    qStartPose << -1.99603, 1.4308, -1.12779, -2.29557, 1.78088, 3.11695, 1.0033, 0.0158568, 0.0343686;
    bool isSamplingSuccess1 = true;
    mVis.setStartJointPose(qStartPose);      /// Show the start state in the viewer.
    pin::forwardKinematics(model,data,qGoalPose);
    pin::updateFramePlacements(model, data);

    // create a DifferentialIk object
    cpproboplan::inverseKinematics::DifferentialIkOptions ikOptions;
    ikOptions.ignore_joint_indices = ignoredJointIds;
    ikOptions.damping = 0.001;
    // ikOptions.min_step_size = 0.025;
    // ikOptions.max_step_size = 0.1;
    ikOptions.rng_seed = 113;
    ikOptions.deltaTime = 0.1;
    // Eigen::VectorXd q_neutral;
    // pin::neutral(model,q_neutral);
    cpproboplan::inverseKinematics::DifferentialIk diffIkObj =  cpproboplan::inverseKinematics::DifferentialIk
                (
                    model, 
                    collisionModel,
                    ikOptions,
                    qStartPose
                );
    pin::forwardKinematics(model,data,qStartPose);
    mVis.showStartAndGoalEEPose(qStartPose, qGoalPose); // TODO: 
    std::cerr << "Press ENTER to start solving IK..." << std::endl;
    cpproboplan::crWaitForKeyPress();
    
    bool isSuccess = diffIkObj.solve(targetFrame,oMdes,qStartPose);

    pin::Model::ConfigVectorType solutionJointVector = diffIkObj.getResult();
    // what is the pose given by solutionJointVector
    pin::forwardKinematics(model,data,solutionJointVector);
    pin::updateFramePlacements(model, data);
    const pin::SE3 iMd = data.oMf[targetFrameId].actInv(oMdes);
    Eigen::Matrix<double,6,1> xi = pinocchio::log6(iMd).toVector();
    Eigen::Vector3d p_err = xi.head<3>();   // meters
    Eigen::Vector3d w_err = xi.tail<3>();   // radians
    std::cerr <<"-------------------------final solution-------------"<<std::endl;
    std::cerr << " final pose-error  between the actual pose and  and calcluated pose is : " << p_err.norm() << " meters and "  << w_err.norm() << " rads" <<std::endl;


    /// Optional pause before planning (e.g., to inspect start/goal in the viewer).

    /// Access underlying Meshcat instance if needed for custom visualization.
    std::shared_ptr<Meshcat> meshcatPointer = mVis.getMeshcatPtr();
    
    // /**
    //  * @brief Step through the solution in the viewer, waiting for ENTER at each waypoint.
    //  * @details Each iteration maps a waypoint to an Eigen config vector for the visualizer.
    //  */
    std::cerr << "Press ENTER to move the arm to the found solution Joint..." << std::endl;
    cpproboplan::crWaitForKeyPress();
    mVis.stepSim(solutionJointVector);
    std::cerr << "Press ENTER to exit..." << std::endl;
    cpproboplan::crWaitForKeyPress();
    
    return 0;
}
