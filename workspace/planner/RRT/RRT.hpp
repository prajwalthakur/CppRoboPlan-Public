/**
 * @file RRT.hpp
 * @author Prajwal Thakur (you@domain.com)
 * @brief  Rapidly-expanding Random Tree (RRT) planner.
 *
 * This is a sampling-based motion planner that finds collision-free paths from a start to a goal configuration.
 *
 * good resources:
 * * Original RRT paper: https://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf
 * * Sebastian Castro's github pyroboplan Repository : https://github.com/sea-bass/pyroboplan
 * @version 0.1
 * @date 2025-08-22
 * * @copyright Copyright (c) 2025
 * */

#pragma once
#include <iostream>
#include "/root/workspace/src/planner/base/KdTree.hpp"
#include "/root/workspace/src/core/core.h"
#include "RRTPlannerOptions.h"
#include "/root/workspace/src/planner/base/PlannerBase.hpp"

namespace cpproboplan::planner
{
    /**
     * @class RRT
     * @brief Implements the Rapidly-exploring Random Tree (RRT) motion planning algorithm.
     * * RRT is a sampling-based planner that constructs a tree to explore the state space. It is particularly
     * effective for high-dimensional problems. This class provides the main interface for setting up and
     * solving a planning problem using the RRT algorithm.
     */
    class RRT : public PlannerBase
    {
        public : 
            /**
             * @brief Default constructor for the RRT planner.
             */
            RRT();
            
            /**
             * @brief Parameterized constructor for the RRT planner.
             * * @param spaceType The type of planning space (joint or Euclidean).
             * @param model Pinocchio robot model.
             * @param collisionModel Pinocchio robot collision model.
             * @param plannerOptions Configuration options for the RRT algorithm.
             */
            RRT(const plSpaceType& spaceType, 
                const pin::Model& model,
                const pin::GeometryModel& collisionModel,
                const RRTPlannerOptions& plannerOptions);
            
            /**
             * @brief Destructor for the RRT planner.
             */
            ~RRT();
            
            /**
             * @brief Sets the maximum number of samples to generate.
             * * @param maxSamplingNum The maximum number of samples.
             */
            void setMaxSamplingNum(const std::size_t maxSamplingNum);

            /**
             * @brief Sets the goal sampling rate.
             * * @param goalSamplingRate The probability of sampling the goal node directly (0.0 to 1.0).
             */
            void setGoalSamplingRate(double goalSamplingRate);

            /**
             * @brief Sets the expansion distance for extending the tree.
             * * @param expandDist The distance by which to extend a new node from its nearest neighbor.
             */
            void setExpandDist(double expandDist);

            /**
             * @brief Solves the planning problem from a start to a goal pose.
             * * @param startPose A vector of doubles representing the starting configuration.
             * @param goalPose A vector of doubles representing the goal configuration.
             * @return true If a path is found.
             * @return false If a path is not found within the given constraints.
             */
            bool solve(std::vector<double>& startPose, 
            std::vector<double>& goalPose) override;
        
        private:
            /**
             * @brief Initializes the planner, including models and data structures.
             */
            void init();

        private:
            RRTPlannerOptions mPlannerOptions; /**< Configuration options for the RRT planner. */
            plSpaceType mSpaceType; /**< The type of planning space. */
            plSharedStateSpacePtrF mPlStateSpacePtr{nullptr}; /**< Shared pointer to the state space information. */
            kdTreeUniquePtr mKdTree{nullptr}; /**< Unique pointer to the KD-Tree for nearest neighbor search. */
            pin::Model mPinModel; /**< Pinocchio robot model. */
            pin::Data mData; /**< Pinocchio robot data. */
            pin::GeometryModel mCollisionModel; /**< Pinocchio robot collision model. */
            pin::GeometryData mCollisionData; /**< Pinocchio robot collision data. */

            cpproboplan::crRandVecGenerator<double> mRandomVecGenerator; /**< Random vector generator. */
            cpproboplan::crRandomGenerator<double> mRandomNumGenerator; /**< Random number generator for goal biasing. */
    };

}