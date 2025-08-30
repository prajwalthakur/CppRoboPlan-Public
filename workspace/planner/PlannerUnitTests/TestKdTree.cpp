
#include "/root/workspace/src/core/core.h"
#include "/root/workspace/src/core/coreTest/TestFixture.hpp"
#include "/root/workspace/src/planner/base/KdTree.hpp"
#include "/root/workspace/src/planner/base/Node.h"
#include "/root/workspace/src/planner/base/StateSpaceInformation.h"
#include "/root/workspace/src/planner/base/StateInformation.h"
#include "/root/workspace/src/core/Memory.hpp"


using namespace cpproboplan::planner;





template <template <typename> class T, typename Q>
using plNodePtr = plNode<T, Q>*;

using plSharedNodePtrF       = cpproboplan::rplSharedPtr<plNode<plJointState, double>>;
using plSharedStateSpacePtrF = cpproboplan::rplSharedPtr<plStateSpace<plJointState, double>>;
using kdTreeUniquePtr        = cpproboplan::rplUniquePtr<plKdTree<plJointState,double>>;


// Test fixture for kdTree
class TeskdTree : public rplTestFixture
{
public:
    TeskdTree() {
        // construct the state space
        plStateSpacePtr = std::make_shared<plStateSpace<plJointState, double>>(dim);

        // construct nodes, passing state space and data
        node1 = std::make_shared<plNode<plJointState, double>>(plStateSpacePtr, node1data);
        node2 = std::make_shared<plNode<plJointState, double>>(plStateSpacePtr, node2data);
        node3 = std::make_shared<plNode<plJointState, double>>(plStateSpacePtr, node3data);
        node4 = std::make_shared<plNode<plJointState, double>>(plStateSpacePtr, node4data);
    }

    ~TeskdTree() {}

protected:
    std::size_t dim = 2;

    plSharedNodePtrF node1{nullptr};
    plSharedNodePtrF node2{nullptr};
    plSharedNodePtrF node3{nullptr};
    plSharedNodePtrF node4{nullptr};

    plSharedStateSpacePtrF plStateSpacePtr{nullptr};

    std::vector<double> node1data = {1.0, 0.0};
    std::vector<double> node2data = {-1.0, 0.0};
    std::vector<double> node3data = {0.0, 0.0};
    std::vector<double> node4data = {0.0, 1.0};
};


////////////////////////////////////////////////////////////////////////////////
// Test No: 1 - check Node Creation and destruction

rplTest(TeskdTree, TestNodeCreation)
{
    EXPECT_NE(node1, nullptr);
    EXPECT_NEAR(node1->getState()[0], 1.0, 1e-9); 
    

}

////////////////////////////////////////////////////////////////////////////////
// Test No: 2 - check Adding Node in KD Tree

rplTest(TeskdTree, TestkdTreeCreation)
{
    kdTreeUniquePtr kdTree = std::make_unique<plKdTree<plJointState,double>>(dim);
    EXPECT_EQ(kdTree->getDim(), dim);
    EXPECT_EQ(kdTree->getRoot(), nullptr);
    EXPECT_EQ(kdTree->getDepth(), 0);
    
}

////////////////////////////////////////////////////////////////////////////////
// Test No: 3 - checking Nearest Neighbour function

rplTest(TeskdTree, TestkdTreeAddingNodes)
{
    kdTreeUniquePtr kdTree = std::make_unique<plKdTree<plJointState,double>>(dim);
    kdTree->add(node1);
    auto nn = kdTree->getRoot();
    ASSERT_NE(nn, nullptr)<< " root is a null pointer"; // gtest assertion, avoids segfault
    EXPECT_EQ(nn->idx,0);
    auto node = kdTree->getPlNodeAtindexi(0);
    EXPECT_TRUE(node==node1);
    auto NNode = kdTree->searchNN(node1);
    EXPECT_TRUE(NNode !=nullptr);
    EXPECT_TRUE(NNode == node1);
    NNode = kdTree->searchNN(node2);
    EXPECT_TRUE(NNode==node1);
    kdTree->add(node3);
    NNode = kdTree->searchNN(node2);
    EXPECT_TRUE(NNode==node3);
}

////////////////////////////////////////////////////////////////////////////////
// Test No: 4 - checking balancing

rplTest(TeskdTree, TestkdTreeNotBalance)
{
    kdTreeUniquePtr kdTree = std::make_unique<plKdTree<plJointState,double>>(dim);
    kdTree->setRebalanceRatioThreshold(2.0);
    kdTree->add(node1);
    EXPECT_EQ(kdTree->getDepth(), 0)<< "1. depth is : " << kdTree->getDepth();
    kdTree->add(node3);
    EXPECT_EQ(kdTree->getDepth(), 1)<< "2. depth is : " << kdTree->getDepth();
    EXPECT_TRUE(kdTree->getRebalanceRatio()>=1.0)<< "2. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;;
    kdTree->add(node2);
    EXPECT_EQ(kdTree->getDepth(), 2)<< "3 . depth is : " << kdTree->getDepth();
    EXPECT_TRUE(kdTree->getRebalanceRatio()<=1.0)<< "3. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;
    kdTree->add(node4);
    EXPECT_EQ(kdTree->getDepth(), 3)<< "4. depth is : " << kdTree->getDepth();
    EXPECT_TRUE(kdTree->getRebalanceRatio()>=1.5)<< "4. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;
}


////////////////////////////////////////////////////////////////////////////////
// Test No: 5 - checking balancing

rplTest(TeskdTree, TestkdTreeBalance)
{
    kdTreeUniquePtr kdTree = std::make_unique<plKdTree<plJointState,double>>(dim);
    kdTree->setRebalanceRatioThreshold(1.1);
    kdTree->add(node1);
    EXPECT_EQ(kdTree->getDepth(), 0)<< "1. depth is : " << kdTree->getDepth();
    kdTree->add(node3);
    EXPECT_EQ(kdTree->getDepth(), 1)<< "2. depth is : " << kdTree->getDepth();
    EXPECT_TRUE(kdTree->getRebalanceRatio()>=1.0)<< "2. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;;
    kdTree->add(node2);
    EXPECT_EQ(kdTree->getDepth(), 2)<< "3 . depth is : " << kdTree->getDepth();
    EXPECT_TRUE(kdTree->getRebalanceRatio()<=1.0)<< "3. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;
    kdTree->add(node4);
    EXPECT_EQ(kdTree->getDepth(), 2)<< "4. depth is : " << kdTree->getDepth();
    EXPECT_NEAR(kdTree->getRebalanceRatio(),1.0,1e-9)<< "4. Rebalance ratio was: " << kdTree->getRebalanceRatio()<<" mcalcRatio "<<kdTree->mcalcRatio;
}