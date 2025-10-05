#pragma once
#include <chrono>
#include <thread>

#include <drake/geometry/meshcat.h>
#include <drake/geometry/shape_specification.h>   // Mesh, Box, etc.
#include <drake/math/rigid_transform.h>
#include <pinocchio/algorithm/frames.hpp>
#include "/root/workspace/src/core/core.h"
#include "VisualizerOptions.h"
namespace pin = pinocchio;
using drake::geometry::Meshcat;
using drake::geometry::Mesh;
using drake::math::RigidTransformd;
namespace cpproboplan::visualizer
{

    class visJointTrajectory
    {
        public:
            // Constructor
            visJointTrajectory()=default;
            // Destructor
            ~visJointTrajectory()=default;
            /**
             * @brief Construct a new vis Joint Trajectory object
             * 
             * @param model 
             * @param visualModel 
             * @param collisionModel 
             * @param visualizerOptions 
             */
            visJointTrajectory(const pin::Model& model,
                const pin::GeometryModel& visualModel,
                const pin::GeometryModel& collisionModel,
                const VisualizerOptions& visualizerOptions);
            /**
             * @brief Get the Meshcat Ptr object
             * 
             * @return rplSharedPtr<Meshcat> 
             */
            rplSharedPtr<Meshcat> getMeshcatPtr();
            /**
             * @brief Set the Start Time object
             * 
             * @param currTime 
             */
            void setStartTime(const std::chrono::high_resolution_clock::time_point currTime);
            /**
             * @brief Return the duration elpased since the start time has been set.
             * 
             * @param currTime 
             * @return double 
             */
            double elapsedTime(const std::chrono::high_resolution_clock::time_point currTime);
            /**
             * @brief Set the Start Joint Pose object
             * 
             * @param JointState 
             */
            void setStartJointPose(const rplState& JointState);
            /**
             * @brief show the start and goal End effector goal pose in drake meshcat visualizer.
             * 
             * @param startJointPose 
             * @param goalPose 
             */
            void showStartAndGoalEEPose(const rplState& startJointPose, const rplState& goalJointPose);
            /**
             * @brief Step the simulator to the next joint-state.
             * 
             * @param JointState 
             */
            void stepSim(const rplState& JointState);
                        
            void stepSim(const rplState& JointState, pin::Model::TangentVectorType JointSpeed);

            void stepSim(const rplState& JointState, pin::Model::TangentVectorType JointSpeed, pin::Model::TangentVectorType Jointacc);
            /**
             * @brief Add the box in drake mesh cat.
             * 
             * @param box 
             * @param go 
             */
            void addCoalBoxToMeshacat(const rplSharedPtr<coal::Box>& box, const pin::GeometryModel::GeometryObject& go);
            /**
             * @brief Add the sphere in drake mesh cat.
             * 
             * @param sphere 
             * @param go 
             */
            void addCoalSphereToMeshCat(const rplSharedPtr<coal::Sphere>& sphere, const pin::GeometryModel::GeometryObject& go);
            /**
             * @brief To be used to show the end effector path.
             * 
             */
            void updateEEPath();
        private:
            void init();
            void updateMeshcatTransforms();
            void AddFrameAxes(
                    const std::string& path,
                    const drake::math::RigidTransformd& X_WF,
                    const double axis_length = 0.2,
                    const double axis_radius = 0.005) ;
        
        private:
            VisualizerOptions mVisualizerOptions;
            pin::Model mPinModel;
            pin::Data mData;
            pin::GeometryModel mVisualModel; 
            pin::GeometryData mVisualData; 
            pin::GeometryModel mCollisionModel; 
            pin::GeometryData mCollisionData; 
            double mVisualRate;
            double mSleepDuration;
            double mStartTime;
            rplSharedPtr<Meshcat> mMeshCatPtr;
            std::vector<Eigen::Vector3d> mEEPath; // Not used yet.
            

    };



}