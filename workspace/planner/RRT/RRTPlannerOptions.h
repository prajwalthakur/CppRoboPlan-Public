#pragma once
#include <limits>   // for std::numeric_limits
#include <optional> // for std::optional
namespace cpproboplan::planner
{
    /**
     * @brief Options for Rapidly-exploring Random Tree (RRT) planning.
     *
     * This class encapsulates configurable parameters used in different
     * RRT-based algorithms such as RRT, RRT-Connect, Bidirectional RRT,
     * and RRT*. It provides defaults suitable for general use but allows
     * customization for specific planning problems.
     */
    class RRTPlannerOptions
    {
    public:
        /**
         * @brief Sampling distribution type options:(uniform_real,normal)
         * 
         */
        std::string distribution_type;
        /**
         * @brief padding for shrink joint limit  
         * 
         */
        double joint_limit_padding;
        /**
         * @brief maximum number of tries to get the collision free sample
         * 
         */
        
         std::size_t max_tries;
         /**
          * @brief  margin, in meters, to apply when calculating distance to nearest collision.
          * 
          */
          double collision_safety_margin;
        /**
         * @brief Maximum joint configuration step size for collision checking
         *        along path segments.
         */
        double max_step_size;

        /**
         * @brief Maximum angular distance, in radians, for connecting nodes.
         */
        double max_connection_dist;

        /**
         * @brief If true, enables the RRTConnect algorithm, which incrementally
         *        extends the most recently sampled node until an invalid state is reached.
         */
        bool rrt_connect;

        /**
         * @brief If true, uses bidirectional RRTs from both start and goal nodes.
         *        Otherwise, only grows a tree from the start node.
         */
        bool bidirectional_rrt;

        /**
         * @brief If true, enables the RRT* algorithm to shortcut node connections during planning.
         *        This in turn will use the `max_rewire_dist` parameter.
         */
        bool rrt_star;

        /**
         * @brief Maximum angular distance, in radians, to consider rewiring nodes for RRT*.
         *        If set to infinity, all nodes in the tree will be considered for rewiring.
         */
        double max_rewire_dist;

        /**
         * @brief Maximum planning time, in seconds.
         */
        double max_planning_time;

        /**
         * @brief Optional random number generator seed.
         *        If provided, ensures deterministic planning results.
         */
        std::size_t rng_seed;

        /**
         * @brief If true, return as soon as a solution is found.
         *        Otherwise, continue building the tree until `max_planning_time` is reached.
         */
        bool fast_return;

        /**
         * @brief Probability of sampling the goal configuration directly.
         *        Useful for biasing the planner towards convergence.
         */
        double goal_biasing_probability;

        /**
         * @brief default constructor
         */
        //RRTPlannerOptions()=default;

        /**
         * @brief default destructor;
         */        
        ~RRTPlannerOptions()=default;

        /**
         * @brief Constructor with default values for all planner options.
         *
         * @param max_step_size_ Maximum step size for collision checking. Default: 0.05
         * @param max_connection_dist_ Maximum distance for node connections. Default: 0.5
         * @param rrt_connect_ Enable RRTConnect. Default: false
         * @param bidirectional_rrt_ Enable bidirectional RRT. Default: false
         * @param rrt_star_ Enable RRT*. Default: false
         * @param max_rewire_dist_ Maximum rewiring distance for RRT*. Default: infinity
         * @param max_planning_time_ Maximum planning time in seconds. Default: 10.0
         * @param rng_seed_ Optional RNG seed. Default: std::nullopt
         * @param fast_return_ Return immediately when a solution is found. Default: true
         * @param goal_biasing_probability_ Probability of sampling the goal. Default: 0.0
         * @param distribution_type_ Sampling Distribution Type. Default: uniform_real
         * @param seed_   A vector of seed for sampling random numbers. size = dimension of sampling space
         * @param joint_limit_padding_ A double for adding the constant padding for joint limits
         * @param max_tries_  Maximum number of tries to get a collision free random sample
         * @param collision_safety_margin_ safety margin for collision checking
         */
        RRTPlannerOptions(
            double max_step_size_ = 0.05,
            double max_connection_dist_ = 0.5,
            bool rrt_connect_ = false,
            bool bidirectional_rrt_ = false,
            bool rrt_star_ = false,
            double max_rewire_dist_ = std::numeric_limits<double>::infinity(),
            double max_planning_time_ = 10.0,
            std::size_t rng_seed_ = 512,
            bool fast_return_ = true,
            double goal_biasing_probability_ = 0.15,
            std::string distribution_type_ = "uniform_real",
            double joint_limit_padding_ = 0.0,
            std::size_t  max_tries_ = 100,
            double collision_safety_margin_ = 0.0
        )
            : max_step_size(max_step_size_),
            max_connection_dist(max_connection_dist_),
            rrt_connect(rrt_connect_),
            bidirectional_rrt(bidirectional_rrt_),
            rrt_star(rrt_star_),
            max_rewire_dist(max_rewire_dist_),
            max_planning_time(max_planning_time_),
            rng_seed(rng_seed_),
            fast_return(fast_return_),
            goal_biasing_probability(goal_biasing_probability_),
            distribution_type(distribution_type_),
            joint_limit_padding(joint_limit_padding_),
            max_tries(max_tries_),
            collision_safety_margin(collision_safety_margin_)
        {}
    };


}