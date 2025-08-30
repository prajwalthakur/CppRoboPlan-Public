/**
 * @file KdTree.hpp
 * @author Prajwal Thakur
 * @brief  KD Tree Data Structure for finding the nearest neighbour of a node, used in Sampling based Planning.
 * Good Resources:
 * * Efficient Nearest Neighbor Searching for Motion Planning Paper : https://lavalle.pl/papers/AtrLav02.pdf  
 * * chapter 8 and 9  for comprehensive review on KD-Tree: Advanced Algorithms and Data Structure by Marcello La Rocca
 * * Useful Implemetation Details on KD tree : https://github.com/medalotte/sampling-based-planners
 * @version 0.1
 * @date 2025-08-22
 * * @copyright Copyright (c) 2025
 * */

#pragma once
#include <vector>
#include <numeric>
#include <algorithm>
#include <limits>
#include "Node.h"
#include "DistanceMetric.h"
#include "StateSpaceInformation.h"

namespace cpproboplan::planner
{
    /**
     * @struct kdTreeNode
     * @brief A node in the KD-Tree data structure.
     * * This struct represents a single node in the KD-Tree. It holds an index to the corresponding
     * planning node (`plNode`), the axis used for splitting, and pointers to its parent and children
     * within the KD-Tree structure.
     */
    struct kdTreeNode
    {   
        int idx{-1}; /**< Index of the corresponding planning node (plNode) in the mPlNodes vector. */
        int axis{-1}; /**< The axis (dimension) used to split this node. */
        /**
         * @brief Index of the parent node in the KD Tree structure.
         * * This is different from the parent-child relationship in a sampling-based planning
         * algorithm, which is based on nearest neighbors. This refers to the tree's
         * hierarchical structure.
         */
        int parentIdx{-1}; 
        cpproboplan::rplUniquePtr<kdTreeNode> child_r{nullptr}; /**< Unique pointer to the right child. */
        cpproboplan::rplUniquePtr<kdTreeNode> child_l{nullptr}; /**< Unique pointer to the left child. */
    };

    using kdNodePtr = kdTreeNode*; /**< A raw pointer to a kdTreeNode. */
    using kdUniqueNodePtr = cpproboplan::rplUniquePtr<kdTreeNode>; /**< A unique pointer to a kdTreeNode. */
    using kdWkNodePtr = cpproboplan::rplwkPtr<kdTreeNode>; /**< A weak pointer to a kdTreeNode. */

    /**
     * @class plKdTree
     * @brief A K-Dimensional Tree implementation for efficient nearest neighbor search.
     * * This class provides an implementation of a KD-Tree, a space-partitioning data structure
     * used for organizing points in a k-dimensional space. It is particularly useful in
     * sampling-based motion planning algorithms like RRT to quickly find the nearest neighbor
     * of a new node.
     * * @tparam T A template parameter for the state representation type.
     * @tparam Q A template parameter for the configuration space representation type.
     */
    template < template <typename> class T,typename Q>
    class plKdTree : public plNode<T,Q>
    {
        public:
            using plNodePtr =   plNode<T,Q>*; /**< Raw pointer type for a planning node. */
            using plWkNodePtr = cpproboplan::rplwkPtr<plNode<T,Q>>; /**< Weak pointer type for a planning node. */
            using plSharedNodePtr = cpproboplan::rplSharedPtr<plNode<T,Q>>; /**< Shared pointer type for a planning node. */
            using plUniqueNodePtr = cpproboplan::rplUniquePtr<plNode<T,Q>>; /**< Unique pointer type for a planning node. */
            
            plKdTree() = default; /**< Default constructor. */

            /**
             * @brief Constructor for the KD-Tree.
             * @param dim The dimension of the state space.
             */
            explicit plKdTree(const size_t dim);
            
            /**
             * @brief Destructor for the KD-Tree.
             */
            ~plKdTree();
            
            /**
             * @brief Adds a new node to the KD-Tree.
             * * This method inserts a new `plNode` into the KD-Tree structure.
             * @param node The shared pointer to the new planning node to add.
             * @param parentPtr A raw pointer to the parent node in the planning tree (not the KD-Tree).
             */
            void add(plSharedNodePtr node, plNodePtr parentPtr = nullptr);

            /**
             * @brief Initializes the KD-Tree.
             * * This function is typically called to build the tree from an initial set of nodes.
             */
            void init();
            
            /**
             * @brief Searches for the nearest neighbor of a given node.
             * * This performs an efficient search to find the closest node in the tree to the query node.
             * @param node The shared pointer to the query node.
             * @return A shared pointer to the nearest neighbor node.
             */
            plSharedNodePtr searchNN(const plSharedNodePtr& node); 
            
            /**
             * @brief Searches for all neighbors within a specified radius.
             * * This method finds all nodes in the KD-Tree that are within a given radius of the query node.
             * @param node The shared pointer to the query node.
             * @param radius The radius to search within.
             * @return A vector of shared pointers to the found neighbor nodes.
             */
            std::vector<plSharedNodePtr> searchNBHD(const plSharedNodePtr& node, const double& radius);

            /**
             * @brief Searches for all leaf nodes in the KD-Tree.
             * @return A vector of shared pointers to the leaf nodes.
             */
            std::vector<plSharedNodePtr> searchLeafs();

            /**
             * @brief Gets the dimension of the state space.
             * @return The dimension.
             */
            std::size_t getDim();

            /**
             * @brief Gets a raw pointer to the root of the KD-Tree.
             * @return A raw pointer to the root node.
             */
            kdNodePtr getRoot();

            /**
             * @brief Gets the current depth of the KD-Tree.
             * @return The depth.
             */
            int getDepth();

            /**
             * @brief Gets the planning node at a specified index.
             * @param idx The index of the node in the internal `mPlNodes` vector.
             * @return A shared pointer to the planning node.
             */
            plSharedNodePtr getPlNodeAtindexi(std::size_t idx);

            /**
             * @brief Gets the rebalance ratio.
             * @return The current rebalance ratio.
             */
            double getRebalanceRatio();

            /**
             * @brief Sets the threshold for the rebalance ratio.
             * @param reRatio The new rebalance ratio threshold.
             */
            void setRebalanceRatioThreshold(double reRatio);

            double mcalcRatio{0.0};


        private:
            double REBALANCE_RATIO{1.7}; /**< Threshold ratio for triggering a tree rebalance. */
            kdUniqueNodePtr mRoot{nullptr}; /**< Unique pointer to the root of the KD-Tree. */
            std::vector<plSharedNodePtr> mPlNodes; /**< A vector of shared pointers to all planning nodes. */
            std::vector<kdNodePtr> mKdTreeNodes; /**< A vector of raw pointers to the KD-Tree nodes. */
            rplSharedPtr<plStateSpace<T,Q>> mStateSpacePtr; /**< Shared pointer to the state space information. */
            int mDepth{0}; /**< Current depth of the KD-Tree. */
            std::size_t mDim{0}; /**< Dimension of the state space. */
            
        private:
            /**
             * @brief Recursively clears the KD-Tree.
             * * This private helper function deallocates the KD-Tree nodes. It does not
             * affect the planning nodes themselves.
             * @param node A unique pointer to the current node to be cleared.
             */
            void clearRec(kdUniqueNodePtr & node);
            
            /**
             * @brief Recursively builds the KD-Tree from scratch.
             * @param indices A vector of indices to be included in the subtree.
             * @param offset The starting offset in the indices vector.
             * @param nPoints The number of points in the current subtree.
             * @param depth The current depth of the recursion.
             * @param parentIdx The index of the parent node.
             * @return A unique pointer to the root of the newly built subtree.
             */
            kdUniqueNodePtr buildRec(std::vector<int>& indices, const int& offset, const int& nPoints, const int& depth, const int& parentIdx);
            
            /**
             * @brief Recursively inserts a new node into the KD-Tree.
             * @param root The unique pointer to the root of the current subtree.
             * @param newNodeIndex The index of the new node to insert.
             * @param depth The current depth of the recursion.
             * @param idx The index of the current node in the `mKdTreeNodes` vector.
             * @return A unique pointer to the root of the modified subtree.
             */
            kdUniqueNodePtr insertRec(kdUniqueNodePtr& root, const int& newNodeIndex, const int& depth, const int& idx);
            
            /**
             * @brief Recursively searches for the nearest neighbor.
             * * This private recursive helper function is used by `searchNN`.
             * @param query The shared pointer to the query node.
             * @param node The unique pointer to the current node being examined.
             * @param closestNode A reference to the shared pointer that will hold the result.
             * @param minDist A reference to the variable that will hold the minimum distance found so far.
             */
            void searchNNRec(const plSharedNodePtr& query, const kdUniqueNodePtr& node, plSharedNodePtr& closestNode, double& minDist) const;

            /**
             * @brief Recursively searches for neighbors within a radius.
             * * This private recursive helper function is used by `searchNBHD`.
             * @param query The shared pointer to the query node.
             * @param node The unique pointer to the current node being examined.
             * @param nearNodes A vector to store the found neighbors.
             * @param radius The search radius.
             */
            void searchNBHDRec(const plSharedNodePtr& query, const kdUniqueNodePtr& node, std::vector<plSharedNodePtr>& nearNodes, const double& radius)const;
    };
} // namespace cpproboplan::planner
    
#include "KdTree.tpp"