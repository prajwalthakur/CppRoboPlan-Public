// Idea is to seperate the state class (which has the core defination of robot/body state) than its allocation.
// i.e we cand define the state but to allocate it in a "space" we can request plStateSpace class to return
// a pointer of type state

#pragma once
#include "/root/workspace/src/core/core.h"
#include "StateInformation.h"

namespace cpproboplan::planner
{
    /**
     * @class plStateSpace
     * @brief A class for allocating state objects within a defined planning space.
     * * This class decouples the definition of a state (e.g., joint state, Euclidean state) from
     * its allocation. A user can request an instance of a specific state type from this class,
     * which handles the memory management and returns a unique pointer.
     * * @tparam T A template template parameter representing the state type (e.g., plJointState, plEuclidState).
     * @tparam Q The scalar type used to represent the state's components (e.g., double).
     */
    template < template <typename> class T,typename Q>
    class plStateSpace
    {
        public:
            /**
             * @brief Default constructor for plStateSpace.
             */
            plStateSpace() = default;

            /**
             * @brief Parameterized constructor for plStateSpace.
             * @param dim The dimension of the state space.
             */
            plStateSpace(std::size_t dim):mDim(dim){}

            /**
             * @brief Default destructor for plStateSpace.
             */
            ~plStateSpace() = default;
            
            //---------------------------------------------

            /**
             * @brief Sets the dimension of the state space.
             * @param dim The dimension to set.
             */
            void setDimension(size_t dim){mDim = dim;}
            
            //---------------------------------------------

            /**
             * @brief Allocates and returns a unique pointer to a new state object.
             * * This function creates a new object of the specified state type `T` and returns a
             * unique pointer to it. The state's dimension is based on the dimension of the
             * `plStateSpace` instance.
             * * @return A unique pointer to the newly allocated state object.
             */
            rplUniquePtr<T<Q>> allocateState()
            {
                // Note: The constructor of T<Q> is responsible for setting the dimension.
                return std::make_unique<T<Q>>();
            }

        private:
            std::size_t mDim{0}; /**< The dimension of the state space. */
    };

}