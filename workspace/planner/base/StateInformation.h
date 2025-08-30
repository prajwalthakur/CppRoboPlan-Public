#pragma once
#include<iostream>
#include <Eigen/Dense>
#include "/root/workspace/src/core/core.h"

namespace cpproboplan::planner
{
    /**
     * @class plState
     * @brief A generic class representing a state in a planning problem.
     * * This class is a base for different types of states, such as joint space or Euclidean space states.
     * It holds the state data as a vector of a templated type.
     * @tparam T The data type of the state's components (e.g., double, float).
     */
    template <typename T>
    class plState
    {
        public:
            /**
             * @brief Default constructor.
             */
            plState():mDim(0){};

            /**
             * @brief Virtual destructor.
             */
            virtual ~plState() = default;

            /**
             * @brief Parameterized constructor.
             * @param dim The dimension of the state vector.
             */
            plState(std::size_t dim): mDim(dim){mState.resize(dim);}

            /**
             * @brief Gets a copy of the state data.
             * @return A copy of the state vector.
             */
            std::vector<T> getState() const { return mState; }

            /**
             * @brief Gets a reference to the state data.
             * @return A modifiable reference to the state vector.
             */
            std::vector<T>& getStateRef() {return mState;}

            /**
             * @brief Gets the dimension of the state.
             * @return The dimension of the state vector.
             */
            std::size_t size() const { return mDim; }

            /**
             * @brief Sets the state data.
             * @param state The new state data vector.
             */
            void setState(std::vector<T> state) {mState = state;}

        private:
            std::size_t mDim; /**< The dimension of the state vector. */
            std::vector<T> mState; /**< The state data. */
    };

    /**
     * @class plJointState
     * @brief A class representing a state in joint space.
     * * This class inherits from plState and is specialized for representing a robot's joint
     * configuration.
     * @tparam T The data type of the joint values.
     */
    template <typename T>
    class plJointState:virtual public plState<T>
    {
        public:
            /**
             * @brief Default constructor.
             */
            plJointState()=default;

            /**
             * @brief Destructor.
             */
            ~plJointState() override = default;

            /**
             * @brief Parameterized constructor.
             * @param dim The dimension of the joint space.
             */
            plJointState(std::size_t dim):plState<T>(dim){return;};
    };

    /**
     * @class plEuclidState
     * @brief A class representing a state in Euclidean space.
     * * This class inherits from plState and is specialized for representing a position or orientation in
     * Euclidean space.
     * @tparam T The data type of the Euclidean coordinates.
     */
    template <typename T>
    class plEuclidState:virtual public plState<T>
    {
        public:
            /**
             * @brief Default constructor.
             */
            plEuclidState()=default;
            
            /**
             * @brief Destructor.
             */
            ~plEuclidState() override = default;
            
            /**
             * @brief Parameterized constructor.
             * @param dim The dimension of the Euclidean space.
             */
            plEuclidState(std::size_t dim):plState<T>(dim){};
    };
}