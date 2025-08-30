#pragma once
#include <vector>
#include <optional>

namespace cpproboplan::inverseKinematics
{
    /**
     * @brief Options for differential IK.
     */
    class DifferentialIkOptions {
    public:
        /**
         * @brief Initializes a set of differential IK options.
         *
         * @param max_iters Maximum number of iterations per try.
         * @param max_retries Maximum number of retries with random restarts.
         *        If set to 0, only the initial state provided will be used.
         * @param max_translation_error Maximum translation error, in meters, to consider IK solved.
         * @param max_rotation_error Maximum rotation error, in radians, to consider IK solved.
         * @param damping Damping value, between 0 and 1, for the Jacobian pseudoinverse.
         *        Setting this to a nonzero value uses Levenberg-Marquardt.
         * @param min_step_size Minimum gradient step size, between 0 and 1.
         *        To use a fixed step size, set both min and max equal.
         * @param max_step_size Maximum gradient step size, between 0 and 1.
         *        To use a fixed step size, set both min and max equal.
         * @param ignore_joint_indices Joints to ignore when solving IK.
         * @param joint_weights Relative weights for joints (higher weight → less movement).
         *        If not specified, all joints are equally weighted.
         * @param rng_seed Random number generator seed (for deterministic results).
         * @param joint_limit_padding padding for sampling Joint 
         * @param distribution_type sampling distribution name: to sample the random number/ vector
         * @param deltaTime integration time in seconds
         */
        DifferentialIkOptions(
            int max_iters = 200,
            int max_retries = 10,
            double max_translation_error = 1e-3,
            double max_rotation_error = 1e-3,
            double damping = 1e-3,
            double min_step_size = 0.1,
            double max_step_size = 0.5,
            std::vector<std::size_t> ignore_joint_indices = {},
            std::optional<std::vector<double>> joint_weights = std::nullopt,
            int rng_seed = 122,
            double joint_limit_padding = 0.0,
            std::string distribution_type = "uniform_real",
            double deltaTime = 0.1,
            int max_tries = 1000,
            double collision_safety_margin = 0.05
        )
            : max_iters(max_iters),
            max_retries(max_retries),
            max_translation_error(max_translation_error),
            max_rotation_error(max_rotation_error),
            damping(damping),
            min_step_size(min_step_size),
            max_step_size(max_step_size),
            ignore_joint_indices(std::move(ignore_joint_indices)),
            joint_weights(std::move(joint_weights)),
            rng_seed(rng_seed),
            joint_limit_padding(joint_limit_padding),
            distribution_type(distribution_type),
            deltaTime(deltaTime),
            max_tries(max_tries),
            collision_safety_margin(collision_safety_margin){}
        int max_iters;
        int max_retries;
        double max_translation_error;
        double max_rotation_error;
        double damping;
        double min_step_size;
        double max_step_size;
        std::vector<std::size_t> ignore_joint_indices;
        std::optional<std::vector<double>> joint_weights;
        int rng_seed;
        std::string distribution_type;
        double deltaTime;
        double joint_limit_padding;
        std::size_t max_tries;
        double collision_safety_margin;

    };



}

