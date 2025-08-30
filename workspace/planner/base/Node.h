#pragma once
#include <iostream>
#include <vector>
#include "/root/workspace/src/core/core.h"
#include "StateSpaceInformation.h"

namespace cpproboplan::planner
{
    /**
     * @class plNode
     * @brief Represents a node in the planning tree.
     * * This class encapsulates a single state in the planning space, along with
     * information about its parent node, which is essential for reconstructing the
     * final path. It is templated to be flexible with different state space representations.
     * * @tparam T A template template parameter representing the state type (e.g., plState).
     * @tparam Q The data type of the state's components (e.g., double, float).
     */
    template < template <typename> class T,typename Q>
    class plNode 
    {
        public:
            /**
             * @brief Default constructor for plNode.
             */
            plNode() = default;

            /**
             * @brief Parameterized constructor for plNode.
             * * @param ss Shared pointer to the state space information.
             * @param data The state data as a vector of Q.
             * @param parent A raw pointer to the parent node.
             */
            plNode(rplSharedPtr<plStateSpace<T,Q>>& ss, std::vector<Q> data, plNode<T,Q>* parent = nullptr) :
                mStateSpace(ss), mParentPtr(parent)
            {
                mStatePtr = mStateSpace->allocateState();
                mStatePtr->setState(data);        
            }

            /**
             * @brief Default destructor for plNode.
             */
            ~plNode() = default;
            
            //-----------------------------------------------------
            
            /**
             * @brief Gets the state data.
             * @return A copy of the state data vector.
             */
            std::vector<Q> getState()
            {
                std::vector<Q> data = mStatePtr->getState(); 
                return data;
            }
            
            //-----------------------------------------------------

            /**
             * @brief Gets a reference to the state data.
             * @return A modifiable reference to the state data vector.
             */
            std::vector<Q>& getStateRef()
            {
                return mStatePtr->getStateRef();
            }            
            //---------------------------------------------------

            /**
             * @brief Gets a pointer to the parent node.
             * @return A raw pointer to the parent node.
             */
            plNode* getParent()
            {
                return mParentPtr;
            }
            
            //-----------------------------------------------------

            /**
             * @brief Sets the state data.
             * @param data The new state data as a vector.
             */
            void setData(std::vector<Q> data)
            {
                mStatePtr->setState(data);
            }
            
            //---------------------------------------------------

            /**
             * @brief Sets the parent node.
             * @param parentPtr A raw pointer to the new parent node.
             */
            void setParent(plNode<T,Q>* parentPtr)
            {
                mParentPtr = std::move(parentPtr);
            }

            plNode<T,Q>* mParentPtr{nullptr}; /**< Raw pointer to the parent node. */
            rplSharedPtr<plStateSpace<T,Q>> mStateSpace; /**< Shared pointer to the state space information. */
            rplUniquePtr<T<Q>> mStatePtr{nullptr}; /**< Unique pointer to the state object. */
            rplNodeName mNodeName; /**< The name of the node. */
            bool isLeaf{false}; /**< Flag to indicate if the node is a leaf in the planning tree. */
    };
}