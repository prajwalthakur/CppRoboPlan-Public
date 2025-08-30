#pragma once
#include <limits>   // for std::numeric_limits
#include <optional> // for std::optional
namespace cpproboplan::visualizer
{
    /**
     * @brief Options for Rapidly-exploring Random Tree (RRT) planning.
     *
     * This class encapsulates configurable parameters used in  visualization.
     */
    class VisualizerOptions
    {
    public:
        /**
         * @brief rate (in Hz) at which we  run the visualizer
         * 
         */
        double visualizer_rate;

        /**
         * @brief End Effector frame Name
         * 
         */
        std::string ee_frame_name;

        /**
         * @brief Radius to visualize start and goal EE-pose
         * 
         */

        double visRadius;

        /**
         * @brief default destructor;
         */        
        ~VisualizerOptions()=default;

        /**
         * @brief Constructor with default values for all planner options.
         *
         * @param visualizer_rate_ (in Hz) at which we  run the visualizer. Default: 50hz
         */
        VisualizerOptions(
            double visualizer_rate_ = 5,
            std::string ee_frame_name_ = "panda_link7_0",
            double visRadius_ = 0.05
        )
            : visualizer_rate(visualizer_rate_),
            ee_frame_name(ee_frame_name_),
            visRadius(visRadius_){}
    };


}