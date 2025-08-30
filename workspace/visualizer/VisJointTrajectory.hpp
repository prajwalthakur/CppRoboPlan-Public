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
            // constructor
            visJointTrajectory();
            
            // destructor
            ~visJointTrajectory();

            visJointTrajectory(const pin::Model& model,
                const pin::GeometryModel& visualModel,
                const pin::GeometryModel& collisionModel,
                const VisualizerOptions& visualizerOptions);

            std::shared_ptr<Meshcat> getMeshcatPtr();

            void setStartTime(const std::chrono::high_resolution_clock::time_point currTime);
            
            double elapsedTime(const std::chrono::high_resolution_clock::time_point currTime);
            
            void updateEEPath();

            void setStartJointPose(pin::Model::ConfigVectorType JointState);
            
            void showStartAndGoalEEPose(pin::Model::ConfigVectorType startJointPose, pin::Model::ConfigVectorType goalPose);
            
            void stepSim(pin::Model::ConfigVectorType JointState);
                        
            void stepSim(pin::Model::ConfigVectorType JointState, pin::Model::TangentVectorType JointSpeed);

            void stepSim(pin::Model::ConfigVectorType JointState, pin::Model::TangentVectorType JointSpeed, pin::Model::TangentVectorType Jointacc);

            void addCoalBoxToMeshacat(const std::shared_ptr<coal::Box>& box, const pin::GeometryModel::GeometryObject& go);

            void addCoalSphereToMeshCat(const std::shared_ptr<coal::Sphere>& sphere, const pin::GeometryModel::GeometryObject& go);

        private:
            void init();
            void updateMeshcatTransforms();
            void AddFrameAxes(
                    const std::string& path,
                    const drake::math::RigidTransformd& X_WF,
                    double axis_length = 0.2,
                    double axis_radius = 0.005) ;
        
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
            std::shared_ptr<Meshcat> mMeshCatPtr;
            std::vector<Eigen::Vector3d> mEEPath;
            

    };



}