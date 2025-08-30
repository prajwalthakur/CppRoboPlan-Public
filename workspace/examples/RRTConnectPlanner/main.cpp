/**
 * @file main.cpp
 * @brief Example program that builds a Panda robot model with Pinocchio, sets up
 *        visualization and collision geometry, samples start/goal joint states,
 *        and solves a joint-space path planning problem using RRT-Connect.
 *
 * This example demonstrates a RRT connect motion-planning pipeline:
 *  - Build kinematic and geometric models from URDF/SRDF.
 *  - Add/prune self-collision pairs and attach custom object collision geometry.
 *  - Sample collision-free start/goal joint configurations (here optionally hard-coded).
 *  - Plan in joint space with an RRT-Connect planner.
 *  - Visualize the resulting trajectory step-by-step in Meshcat.
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

    /// Instantiate the joint-space RRT-Connect planner.
    cpproboplan::planner::plSpaceType spaceType = cpproboplan::planner::plSpaceType::S_JOINTSPACE;
    cpproboplan::planner::RRTConnect RRTPlanner =
        cpproboplan::planner::RRTConnect(spaceType, model, collisionModel, plannerOptions);
    /** @} */


    /** @name Start/Goal state selection
     *  You can either hard-code states or sample collision-free ones.
     *  @{ */
    pin::Model::ConfigVectorType qStartPose(9); ///< 7 Panda joints + 2 finger joints (example)
    // Examples of alternative starts:
    // qStartPose << -2.87136, 1.04315, -2.08852, -2.35162, -0.604917, 1.08625, -0.0948319, 0.0120117, 0.0390873;
    // qStartPose <<  2.50886, 1.46168, -1.32129, -1.45447, -0.896233, 0.393245, -1.1732, 0.0210665, 0.0216019;

    // Hard-coded start pose (assumed within limits):
    qStartPose << -1.99603, 1.4308, -1.12779, -2.29557, 1.78088, 3.11695, 1.0033, 0.0158568, 0.0343686;
    bool isSamplingSuccess1 = true;

    // Optional: stochastic sampling of a collision-free start
    // auto [qStartPose, isSamplingSuccess1] = cpproboplan::crGetRandomCollisionFreeSample(
    //     model, data, collisionModel, collisionData,
    //     plannerOptions.collision_safety_margin, plannerOptions.max_tries, randomVecGenerator);

    /// Show the start state in the viewer.
    mVis.setStartJointPose(qStartPose);
    


    pin::Model::ConfigVectorType qGoalPose(9); ///< Desired goal pose in joint space
    // qGoalPose << 2.50886, 1.46168, -1.32129, -1.45447, -0.896233, 0.393245, -1.1732, 0.0210665, 0.0216019;
    // qGoalPose << -2.87136, 1.04315, -2.08852, -2.35162, -0.604917, 1.08625, -0.0948319, 0.0120117, 0.0390873;

    // Hard-coded goal pose:
    qGoalPose << 2.01141, 1.71381, 2.55119, -2.3794, 1.92879, 2.47939, -2.20627, 0.00275953, 0.0209803;
    bool isSamplingSuccess2 = true;
    mVis.showStartAndGoalEEPose(qStartPose, qGoalPose);
    // Optional: stochastic sampling of a collision-free goal
    // auto [qGoalPose, isSamplingSuccess2] = cpproboplan::crGetRandomCollisionFreeSample(
    //     model, data, collisionModel, collisionData,
    //     plannerOptions.collision_safety_margin, plannerOptions.max_tries, randomVecGenerator);

    if (!isSamplingSuccess1 || !isSamplingSuccess2)
    {
        std::cerr << "couldn't generate collision-free samples" << std::endl;
    }
    else
    {
        std::cerr << "Generated collision-free samples" << std::endl;
    }
    /** @} */

    /// Convert Eigen vectors to std::vector<double> for planner API.
    std::vector<double> startPose(qStartPose.data(), qStartPose.data() + qStartPose.size());
    std::vector<double> goalPose (qGoalPose.data(),  qGoalPose.data()  + qGoalPose.size());
    
    /// Optional pause before planning (e.g., to inspect start/goal in the viewer).
    //std::this_thread::sleep_for(std::chrono::duration<double>(10));

    /**
     * @brief Solve the joint-space motion planning problem.
     * @return True if a path was found; false otherwise.
     * @post On success, use @c getResult() to retrieve the planned path and cost.
     */
    std::cerr << " press Enter to  start planning" << std::endl;
    cpproboplan::crWaitForKeyPress();
    bool isSolved = RRTPlanner.solve(startPose, goalPose);

    if (!isSolved)
    {
        std::cerr << "couldn't solve the path planning problem" << std::endl;
    }
    else
    {
        std::cerr << "solved the path planning problem!" << std::endl;
    }
    
    /** @name Retrieve and report results
     *  Extract the planned path, success flag, and cost from the planner.
     *  @{ */
    auto result = RRTPlanner.getResult();
    std::vector<std::vector<double>>  solPath   = result.path;     ///< Discrete joint-space waypoints
    bool                              isFoundPath = result.isSuccess; ///< Redundant success flag
    double                            cost        = result.cost;      ///< Total Euclidean distance (joint space)
    
    std::cerr << "number of Joints in Path : " << solPath.size() << std::endl;
    std::cerr << "Total Euclidean distance in joint space: " << cost << std::endl;
    /** @} */

    
    // std::this_thread::sleep_for(std::chrono::duration<double>(10));

    /// Access underlying Meshcat instance if needed for custom visualization.
    //std::shared_ptr<Meshcat> meshcatPointer = mVis.getMeshcatPtr();
    
    /**
     * @brief Step through the solution in the viewer, waiting for ENTER at each waypoint.
     * @details Each iteration maps a waypoint to an Eigen config vector for the visualizer.
    */
    for (int i = 0; i < static_cast<int>(solPath.size()); ++i)
    {
        mVis.stepSim(Eigen::Map<pin::Model::ConfigVectorType>(solPath[i].data(), solPath[i].size()));
        std::cerr << "iteration " << i << std::endl;
        std::cerr << "Press ENTER to step to next joint..." << std::endl;
        cpproboplan::crWaitForKeyPress();
    }
    std::cerr<< " goal reached; press Enter to end "<<std::endl;
    cpproboplan::crWaitForKeyPress();;
    return 0;
}
