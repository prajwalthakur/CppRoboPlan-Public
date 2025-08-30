/**
 * @file RRTConnect.hpp
 * @author Prajwal Thakur
 * @brief  RRT-Connect planner.
 *
 * This is a sampling-based motion planner that finds collision-free paths from a start to a goal configuration.
 *
 * good resources:
 * * Original RRT Connect paper: J. J. Kuffner and S. M. LaValle, “RRT-Connect: An Efficient Approach to Single-Query Path Planning,” ICRA 2000.
 * * Sebastian Castro's github pyroboplan Repository : https://github.com/sea-bass/pyroboplan
 * @version 0.1
 * @date 2025-08-22
 * * @copyright Copyright (c) 2025
 * */

#pragma once
#include <iostream>
#include "/root/workspace/src/planner/base/KdTree.hpp"
#include "/root/workspace/src/core/core.h"
#include "RRTConPlannerOptions.h"
#include "/root/workspace/src/planner/base/PlannerBase.hpp"

namespace cpproboplan::planner
{
    /**
     * @class RRTConnect
     * @brief Implements the RRT-Connect motion planning algorithm.
     *
     * This bidirectional planning algorithm builds two RRTs, one from the start and one from the goal,
     * and iteratively attempts to connect them. This often leads to faster convergence and is
     * particularly effective for problems in high-dimensional spaces.
     */
    class RRTConnect : public PlannerBase
    {
        
        public : 
            /**
             * @brief Default constructor for the RRT-Connect planner.
             */
            RRTConnect();
            
            /**
             * @brief Parameterized constructor for the RRT-Connect planner.
             * @param spaceType The type of planning space (joint or Euclidean).
             * @param model Pinocchio robot model.
             * @param collisionModel Pinocchio robot collision model.
             * @param plannerOptions Configuration options for the RRT-Connect algorithm.
             */
            RRTConnect(const plSpaceType& spaceType, 
            const pin::Model& model,
            const pin::GeometryModel& collisionModel, 
            const RRTConPlannerOptions& plannerOptions);
            
            /**
             * @brief Destructor for the RRT-Connect planner.
             */
            ~RRTConnect();
            
            /**
             * @brief Sets the maximum number of samples to generate.
             * @param maxSamplingNum The maximum number of samples.
             */
            void setMaxSamplingNum(const std::size_t maxSamplingNum);

            /**
             * @brief Sets the goal sampling rate.
             * @param goalSamplingRate The probability of sampling the goal node.
             */
            void setGoalSamplingRate(double goalSamplingRate);

            /**
             * @brief Sets the expansion distance for extending the tree.
             * @param expandDist The distance by which to extend a new node from its nearest neighbor.
             */
            void setExpandDist(double expandDist);

            /**
             * @brief Extends a node in one of the RRTs towards a given node.
             * * This function attempts to extend the `parentNode` towards the `node` by a small
             * step. If the extension is collision-free, the new node is added to the tree.
             * @param parentNode The node in the tree to extend from.
             * @param node The node to extend towards.
             * @return true if the node was successfully extended, false otherwise.
             */
            bool extendNode(plSharedNodePtr parentNode, plSharedNodePtr node);

            /**
             * @brief Connects a node to a different tree.
             * * This function is used to attempt a connection from the `parentNode` in one tree
             * to the `node` of the other tree, effectively bridging the gap between them.
             * @param parentNode The node in the current tree.
             * @param node The node in the other tree.
             * @return true if a successful connection is made, false otherwise.
             */
            bool ConnectNode(plSharedNodePtr parentNode, plSharedNodePtr node);

            /**
             * @brief Solves the planning problem from a start to a goal pose.
             * @param startPose A vector of doubles representing the starting configuration.
             * @param goalPose A vector of doubles representing the goal configuration.
             * @return true If a path is found.
             * @return false If a path is not found within the given constraints.
             */
            bool solve(std::vector<double>& startPose, 
            std::vector<double>& goalPose) override;

        private:
            /**
             * @brief Initializes the planner's internal state, including the KD-Trees.
             */
            void init();
        private:
            RRTConPlannerOptions mPlannerOptions; /**< Configuration options for the RRT-Connect planner. */
            plSpaceType mSpaceType; /**< The type of planning space. */
            plSharedStateSpacePtrF mPlStateSpacePtr{nullptr}; /**< Shared pointer to the state space information. */
            
            pin::Model mPinModel; /**< Pinocchio robot model. */
            pin::Data mData; /**< Pinocchio robot data. */
            pin::GeometryModel mCollisionModel; /**< Pinocchio robot collision model. */
            pin::GeometryData mCollisionData; /**< Pinocchio robot collision data. */
            
            
            std::shared_ptr<kdTreeType> mStartPhaseKdTree{nullptr}; /**< The KD-Tree for the start-tree. */
            std::shared_ptr<kdTreeType> mGoalPhaseKdTree{nullptr}; /**< The KD-Tree for the goal-tree. */
            
            
            std::shared_ptr<kdTreeType>  mCurrKdTreePtr{nullptr}; /**< A pointer to the tree currently being extended. */
            std::shared_ptr<kdTreeType>  mOtherKdTreePtr{nullptr}; /**< A pointer to the other tree. */

            cpproboplan::crRandVecGenerator<double> mRandomVecGenerator; /**< Random vector generator for sampling. */
            cpproboplan::crRandomGenerator<double> mRandomNumGenerator; /**< Random number generator for goal biasing. */
    };
}