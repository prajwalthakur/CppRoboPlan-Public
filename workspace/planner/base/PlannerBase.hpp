#pragma once 
#include "/root/workspace/src/core/core.h"
#include "KdTree.hpp"
#include <chrono>

namespace pin = pinocchio;
namespace cpproboplan::planner
{   
    
    /**
     * @brief Type alias for a planning node using a JointState and double data type.
     */
    using plNodeType            =          plNode<plJointState, double>;

    /**
     * @brief Type alias for a shared pointer to a plNodeType.
     */
    using plSharedNodePtr     = cpproboplan::rplSharedPtr<plNode<plJointState, double>>;

    /**
     * @brief Type alias for a shared constant pointer to a plNodeType.
     */
    using plSharedConstNodePtr = cpproboplan::rplSharedPtr<const plNode<plJointState, double>>;

    /**
     * @brief Type alias for a shared pointer to a JointState-based state space.
     */
    using plSharedStateSpacePtrF = cpproboplan::rplSharedPtr<plStateSpace<plJointState, double>>;
    
    /**
     * @brief Type alias for a KD-Tree of plJointState and double types.
     */
    using kdTreeType = plKdTree<plJointState,double>;

    /**
     * @brief Type alias for a unique pointer to a kdTreeType.
     */
    using kdTreeUniquePtr        =  cpproboplan::rplUniquePtr<kdTreeType>;

    /**
     * @enum plSpaceType
     * @brief Enumeration for different types of planning spaces.
     */
    enum class plSpaceType
        {
            S_JOINTSPACE = 0, /**< Planning in joint space. */
            S_EUCLIDSPACE = 1, /**< Planning in Euclidean space. */
        };

    /**
     * @struct plResult
     * @brief Structure to hold the result of a planning query.
     */
    struct plResult
    {
        std::vector<std::vector<double>> path; /**< The computed path as a sequence of state vectors. */
        double cost{0.0}; /**< The cost of the computed path. */
        bool isSuccess{false}; /**< Flag indicating whether a path was successfully found. */
        
        /**
         * @brief Initializes the result struct by clearing all data.
         */
        void init(){
            path.clear();
            cost = 0.0;
            isSuccess = false;
        }
    };
    
    /**
     * @class PlannerBase
     * @brief Abstract base class for all planning algorithms.
     * * This class defines the common interface and shared functionalities for all
     * motion planners, such as managing a KD-Tree, handling results, and performing
     * common operations like collision checking and node steering.
     */
    class PlannerBase
    {
        public:
            /**
             * @brief Default constructor.
             */
            PlannerBase() =default;

            /**
             * @brief Parameterized constructor.
             * @param dim The dimension of the planning space.
             */
            PlannerBase(const std::size_t &dim):mDim{dim}{mResult.init();};

            /**
             * @brief Virtual destructor to ensure proper cleanup of derived classes.
             */
            virtual ~PlannerBase() = default;
            
            PlannerBase(const PlannerBase&) = delete; /**< Deleted copy constructor. */
            PlannerBase& operator=(const PlannerBase&) = delete; /**< Deleted copy assignment operator. */
            PlannerBase(const PlannerBase&&) = delete; /**< Deleted move constructor. */
            PlannerBase& operator=(const PlannerBase&&) = delete; /**< Deleted move assignment operator. */

            /**
             * @brief Sets the start time for duration calculation.
             * @param start The starting time point.
             */
            void setStartTime(const std::chrono::high_resolution_clock::time_point& start);
            
            /**
             * @brief Finds the time elapsed since the start time.
             * @param currTime The current time point.
             * @return The elapsed time in seconds as a double.
             */
            double getElapsedTime(const std::chrono::high_resolution_clock::time_point& currTime);

            /**
             * @brief Virtual base method to solve the planning problem.
             * * This is a pure virtual function that must be implemented by derived classes.
             * @param startPos The start position as a vector of doubles.
             * @param goalPose The goal position as a vector of doubles.
             * @return True if a solution is found, false otherwise.
             */
            virtual bool solve(std::vector<double>& startPos, std::vector<double>& goalPose)=0;
            
            /**
             * @brief Placeholder for setting the problem definition.
             */
            void setProblemDefination(){}

            /**
             * @brief Gets the planning result.
             * @return The plResult struct containing the path, cost, and success status.
             */
            const plResult getResult() const{ return mResult;}

            /**
             * @brief Gets the dimension of the planning space.
             * @return The dimension.
             */
            std::size_t  getDim(){return mDim;}

        protected:
            /**
            * @brief Generates a steered node.
            * * This function generates a new node that is `max_connection_dist` away from `startNode`
            * in the direction of `goalNode`.
            * @param startNode The source node.
            * @param goalNode The destination node.
            * @param max_connection_dist The maximum distance for steering.
            */   
            void generateSteerNode(plSharedNodePtr startNode, plSharedNodePtr goalNode, double max_connection_dist);
            
            /**
            * @brief Placeholder for updating the parent of a node.
            */
            void updateParent(){}

            /**
            * @brief Constructs the final path and fills the result struct.
            * @param finalNode A raw pointer to the final node of the found path.
            * @param isPathFound A boolean indicating if a path was successfully found.
            */
            void constructResult(plNodeType* finalNode, bool isPathFound);
            
            /**
             * @brief Placeholder for a post-processing step.
             * @param max_connection_dist The maximum connection distance.
             */
            void postProcess(double max_connection_dist);

            /**
             * @brief Checks for collisions along a discretized path.
             * * This function checks for collisions between two nodes by discretizing the path
             * and checking each intermediate state for self-collision and environment collision.
             * @param startNode The starting node of the segment.
             * @param goalNode The ending node of the segment.
             * @param model Pinocchio robot model.
             * @param collisionModel Pinocchio robot collision model.
             * @param data Pinocchio robot data.
             * @param collisionData Pinocchio robot collision data.
             * @param max_step_size The maximum step size for discretization.
             * @param collision_safety_margin The safety margin for collision checking.
             * @param stopAtFirstCollision A flag to stop at the first detected collision.
             * @return True if no collision is detected, false otherwise.
             */
            bool discretizeAndCheckCollision(plSharedNodePtr startNode, 
                plSharedNodePtr goalNode, 
                pin::Model& model, 
                pin::GeometryModel& collisionModel, 
                pin::Data& data, 
                pin::GeometryData& collisionData, 
                double max_step_size,
                double collision_safety_margin,
                bool stopAtFirstCollision ); 

            plResult mResult; /**< The result of the planning query. */

        private:
            const std::size_t mDim{0}; /**< The dimension of the planning space. */
            std::chrono::high_resolution_clock::time_point mStartTime; /**< The start time point for timing calculations. */
    };
}