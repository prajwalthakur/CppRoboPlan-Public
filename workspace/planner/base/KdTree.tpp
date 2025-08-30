namespace cpproboplan::planner 
{
    /**
     * @brief Constructs a plKdTree.
     * @param dim The dimension of the state space.
     */
    template <template <typename> class T, typename Q>
    plKdTree<T,Q>::plKdTree(const size_t dim):
         mDim(dim),
         mRoot(nullptr),
         mDepth(0){
            mStateSpacePtr = std::make_shared<plStateSpace<T,Q>>(dim);
         }
    
    //------------------------------------------------------------------

    /**
     * @brief Destructs the plKdTree.
     * * This destructor correctly handles memory management by resetting the unique pointer
     * to the root, which in turn deallocates the entire KD-tree structure. It also clears
     * the vectors holding shared and raw pointers to ensure all resources are released.
     */
    template <template <typename> class T, typename Q>
    plKdTree<T,Q>::~plKdTree() 
    {
        mRoot.reset();           // release KD-tree structure
        mPlNodes.clear();        // release shared_ptrs
        mKdTreeNodes.clear();    // drop raw ptr cache
        mDepth = 0;
    }    
    
    //--------------------------------------------------------------------

    /**
     * @brief Calculates the rebalance ratio of the tree.
     * * The rebalance ratio is a heuristic used to determine when the tree needs to be
     * rebuilt. It is the ratio of the current depth to the theoretical optimal depth
     * for a perfectly balanced tree ($log_2(N)$).
     * @return The calculated rebalance ratio.
     */
    template <template <typename> class T, typename Q>
    double plKdTree<T,Q>::getRebalanceRatio()
    {
        double reRatio = static_cast<double>(mDepth) / std::ceil(std::log2(static_cast<double>(mPlNodes.size())));
        return reRatio;
    }
    
    //------------------------------------------------

    /**
     * @brief Adds a new node to the KD-Tree.
     * * This method first checks if adding the new node unbalances the tree. If the
     * rebalance ratio exceeds the threshold, the entire tree is rebuilt from scratch
     * to ensure optimal search performance. Otherwise, the node is inserted recursively.
     * @param node The shared pointer to the new planning node to add.
     * @param parentPtr A raw pointer to the parent node in the planning tree.
     */
    template <template <typename> class T, typename Q>
    void plKdTree<T,Q>::add( plSharedNodePtr node,plNodePtr parentPtr) 
    {
        if(node->mParentPtr != nullptr){node->mParentPtr->isLeaf = false;}
        mPlNodes.push_back(node);
        mcalcRatio =  getRebalanceRatio();
        if( mcalcRatio >= REBALANCE_RATIO)
        {
            clearRec(mRoot);
            mDepth = 0;
            std::vector<int> indices(mPlNodes.size());
            std::iota(indices.begin(),indices.end(),0);
            mRoot = buildRec(indices, 0 ,(int)mPlNodes.size(), 0, 0 );
        }
        else
        {
            mRoot = insertRec(mRoot,mPlNodes.size()-1,0,0);
        }
        mcalcRatio =  getRebalanceRatio();
        if( getRebalanceRatio() >= REBALANCE_RATIO)
        {
            clearRec(mRoot);
            mDepth = 0;
            std::vector<int> indices(mPlNodes.size());
            std::iota(indices.begin(),indices.end(),0);
            mRoot = buildRec(indices, 0 ,(int)mPlNodes.size(), 0, 0 );
        }
    }

    //----------------------------------------------------
    
    /**
     * @brief Initializes the tree, clearing all nodes and resetting its state.
     */
    template <template <typename> class T, typename Q>
    void plKdTree<T,Q>::init() 
    {
        mRoot = nullptr;
        mDepth = 0;
        mPlNodes.clear(); 
        mKdTreeNodes.clear();
    }

    //---------------------------------------------------------

    /**
     * @brief Searches for the nearest neighbor of a given node.
     * * This function initiates a recursive search for the node closest to the query node.
     * @param node The shared pointer to the query node.
     * @return A shared pointer to the nearest neighbor node found.
     */
    template <template <typename> class T, typename Q>
    typename plKdTree<T,Q>::plSharedNodePtr
    plKdTree<T,Q>::searchNN(const plSharedNodePtr& node) 
    {
        plSharedNodePtr retNode = nullptr;
        auto minDist = std::numeric_limits<double>::max();
        searchNNRec(node,mRoot,retNode,minDist);
        return retNode;
    }

    //--------------------------------------------------------------

    /**
     * @brief Searches for all neighbors within a specified radius.
     * @param node The shared pointer to the query node.
     * @param radius The radius to search within.
     * @return A vector of shared pointers to the found neighbor nodes.
     */
    template <template <typename> class T, typename Q>
    std::vector<typename plKdTree<T,Q>::plSharedNodePtr>
    plKdTree<T,Q>::searchNBHD(const plSharedNodePtr& node, const double& radius)
    {
        std::vector<plSharedNodePtr> nearNodes;
        searchNBHDRec(node, mRoot, nearNodes, radius);
        return nearNodes;
    }
    
    //----------------------------------------------------------

    /**
     * @brief Searches for all leaf nodes in the tree.
     * @return A vector of shared pointers to the leaf nodes.
     */
    template <template <typename> class T, typename Q>
    std::vector<typename plKdTree<T,Q>::plSharedNodePtr>
    plKdTree<T,Q>::searchLeafs()
    {   
        std::vector<plSharedNodePtr> ans;
        for( plSharedNodePtr& node : mPlNodes)
        {
            if(node->isLeaf)
            {
                ans.push_back(node);
            }
        }
        return ans;
    }

    /**
     * @brief Gets the dimension of the state space.
     * @return The dimension.
     */
    template <template <typename> class T, typename Q>
    std::size_t plKdTree<T,Q>::getDim(){return mDim;}

    /**
     * @brief Gets a raw pointer to the root of the KD-Tree.
     * @return A raw pointer to the root node.
     */
    template <template <typename> class T, typename Q>
    kdNodePtr
    plKdTree<T,Q>::getRoot()
    {
        return mRoot.get();
    }

    /**
     * @brief Gets the current depth of the KD-Tree.
     * @return The depth.
     */
    template <template <typename> class T, typename Q>
    int plKdTree<T,Q>::getDepth(){return mDepth;}

    /**
     * @brief Gets the planning node at a specified index.
     * @param idx The index of the node in the internal mPlNodes vector.
     * @return A shared pointer to the planning node.
     */
    template <template <typename> class T, typename Q>
    typename plKdTree<T,Q>::plSharedNodePtr
    plKdTree<T,Q>::getPlNodeAtindexi(std::size_t idx)
    {
        plSharedNodePtr node = mPlNodes[idx];
        return node;
    }
    
    //----------------------------------------
    /**
     * @brief Sets the threshold for the rebalance ratio.
     * @param reRatio The new rebalance ratio threshold.
     */
    template <template <typename> class T, typename Q>
    void plKdTree<T,Q>::setRebalanceRatioThreshold(double reRatio)
    {
        REBALANCE_RATIO = reRatio;
    }
    
    //--------------------------------------------------------------

    /**
     * @brief Recursively clears the KD-Tree structure.
     * @param kdNode A unique pointer to the current node to be cleared.
     */
    template <template <typename> class T, typename Q>
    void plKdTree<T,Q>::clearRec(kdUniqueNodePtr& kdNode)
    {
        if(!kdNode){return;}
        clearRec(kdNode->child_l);
        clearRec(kdNode->child_r);
        kdNode.reset();
    }

    //--------------------------------------------------------------------

    /**
     * @brief Recursively builds the KD-Tree from a set of indices.
     * * This function uses a divide-and-conquer approach. It finds the median element along the
     * current axis using `std::nth_element` and uses it as the root of the current subtree.
     * The process is then repeated for the left and right partitions.
     * @param indices A vector of indices to be included in the subtree.
     * @param offset The starting offset in the indices vector.
     * @param nPoints The number of points in the current subtree.
     * @param depth The current depth of the recursion.
     * @param parentIdx The index of the parent node.
     * @return A unique pointer to the root of the newly built subtree.
     */
    template <template <typename> class T, typename Q>
    kdUniqueNodePtr
    plKdTree<T,Q>::buildRec(std::vector<int>& indices,
        const int& offset, 
        const int& nPoints, 
        const int& depth,
        const int& parentIdx)
    {
       if(nPoints <= 0)
       {
            return nullptr;
       }                   
       mDepth = std::max(depth,mDepth);      
       const int axis = depth%mDim;
       const int midIndex = 0 + ((nPoints - 1))/2;
       auto comp = [&](const int& lhs, const int& rhs)
       {
            const auto& lhsStateVal = mPlNodes[lhs]->getState();
            const auto& rhsStateVal = mPlNodes[rhs]->getState();
            return lhsStateVal[axis] < rhsStateVal[axis] ; 
       };
        std::nth_element(indices.begin()+offset, indices.begin()+offset + midIndex, indices.begin()+offset+nPoints,comp);
        cpproboplan::planner::kdUniqueNodePtr node = std::make_unique<cpproboplan::planner::kdTreeNode>();
        node->idx = indices[offset + midIndex];
        node->parentIdx = parentIdx;
        node->axis = axis;
        node->child_l = buildRec(indices, offset , midIndex, depth+1, node->idx);
        node->child_r = buildRec(indices,offset+midIndex+1, nPoints - (midIndex+1) ,depth+1,node->idx);
        return node;
    }
    
    //------------------------------------------------------------------------

    /**
     * @brief Recursively inserts a new node into the KD-Tree.
     * @param root The unique pointer to the root of the current subtree.
     * @param newNodeIndex The index of the new node to insert.
     * @param depth The current depth of the recursion.
     * @param parentIdx The index of the parent node.
     * @return A unique pointer to the root of the modified subtree.
     */
    template <template <typename> class T, typename Q>
    kdUniqueNodePtr
    plKdTree<T,Q>::insertRec(kdUniqueNodePtr& root, 
        const int& newNodeIndex, 
        const int& depth,
        const int& parentIdx)
    {
        const int axis = depth%mDim;
        if(root == nullptr)
        {
            cpproboplan::planner::kdUniqueNodePtr node = std::make_unique<cpproboplan::planner::kdTreeNode>();
            node->idx = newNodeIndex;
            node->axis = axis;
            node->parentIdx = parentIdx;
            mDepth = std::max(depth,mDepth);
            return std::move(node);
        }
        else
        {
            const auto& newNodeStateVal = mPlNodes[newNodeIndex]->getState();
            const auto& rootNodeStateVal = mPlNodes[root->idx]->getState();
            if( newNodeStateVal[axis] < rootNodeStateVal[axis])
            {
                root->child_l = insertRec(root->child_l, newNodeIndex, depth+1,root->idx);
            }
            else
            {
                root->child_r = insertRec(root->child_r, newNodeIndex, depth+1,root->idx);
            }
            return std::move(root);
        }
    }

    //------------------------------------------------

    /**
     * @brief Recursively searches for the nearest neighbor.
     * @param query The shared pointer to the query node.
     * @param kdNode The unique pointer to the current node being examined.
     * @param closestNode A reference to the shared pointer that will hold the result.
     * @param minDist A reference to the variable that will hold the minimum distance found so far.
     */
    template <template <typename> class T, typename Q>
    void plKdTree<T,Q>::searchNNRec(const plSharedNodePtr& query, 
        const kdUniqueNodePtr& kdNode, 
        plSharedNodePtr& closestNode, 
        double& minDist) const
    {
        if(kdNode==nullptr)
        {   
            if(mRoot == nullptr)
            {
                closestNode = nullptr;
            }
            return;
        }

        plSharedNodePtr guessNode = mPlNodes[kdNode->idx];
        const auto& stateVector1 = query->getState();
        const auto& stateVector2 = guessNode->getState();
        double dist = cpproboplan::distancemetric::Euclidean::calcDistance(stateVector1,stateVector2);
        if(dist < minDist)
        {
            minDist = dist;
            closestNode = guessNode;
        }
        int axis = kdNode->axis;

        int dir = stateVector1[axis] < stateVector2[axis] ? 0:1;

        searchNNRec(query, dir==0 ? kdNode->child_l : kdNode->child_r,closestNode, minDist ) ;

        double projectionDist = cpproboplan::distancemetric::Euclidean::calcProjDistance(stateVector1,stateVector2,axis);
        if(minDist > projectionDist)
        {
            searchNNRec(query, dir==0 ? kdNode->child_r : kdNode->child_l, closestNode, minDist ) ;
        }
    }

    //-------------------------------------

    /**
     * @brief Recursively searches for neighbors within a radius.
     * @param query The shared pointer to the query node.
     * @param kdNode The unique pointer to the current node being examined.
     * @param nearNodes A vector to store the found neighbors.
     * @param radius The search radius.
     */
    template <template <typename> class T, typename Q>
    void plKdTree<T,Q>::searchNBHDRec(const plSharedNodePtr& query, 
        const kdUniqueNodePtr& kdNode, 
        std::vector<plSharedNodePtr>& nearNodes, 
        const double& radius)const
        {
            if(kdNode==nullptr){return;}

            plSharedNodePtr guessNode = mPlNodes[kdNode->idx];
            const auto& stateVector1 = query->getState();
            const auto& stateVector2 = guessNode->getState();
            double dist = cpproboplan::distancemetric::Euclidean::calcDistance(stateVector1,stateVector2);
            
            if(dist < radius)
            {
                nearNodes.push_back(guessNode);
            }
            
            int axis = kdNode->axis;

            int dir = stateVector1[axis] < stateVector2[axis] ? 0:1; //left or right

            searchNBHDRec(query, dir==0 ? kdNode->child_l : kdNode->child_r,nearNodes, radius ) ;

            double projectionDist = cpproboplan::distancemetric::Euclidean::calcProjDistance(stateVector1,stateVector2,axis);
            if(dist > projectionDist)
            {
                searchNBHDRec(query, dir==0 ? kdNode->child_r : kdNode->child_l, nearNodes, radius) ;
            }
        }
} // namespace cpproboplan::planner