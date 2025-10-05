/**
 * @file DifferentialIk.hpp
 * @author Prajwal Thakur (you@domain.com)
 * @brief Differential Ik optimization 

    good resources:
      * Chapter 6: Modern Robotics, Kyvin M Lynch and Frank C Park 2017
      * Sebastian Castro's github pyroboplan Repository : https://github.com/sea-bass/pyroboplan
      * https://scaron.info/robotics/jacobian-of-a-kinematic-task-and-derivatives-on-manifolds.html
 * @version 0.1
 * @date 2025-08-22
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <iostream>
#include <limits.h>
#include <optional>
#include <algorithm>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>     // <-- required
#include "/root/workspace/src/core/core.h"
#include "DifferentialIkOptions.h"
namespace pin = pinocchio;
namespace cpproboplan::inverseKinematics

{

    class DifferentialIk
    {

        public:
            // Constructor.
            DifferentialIk()=default;
            // Destructor.
            ~DifferentialIk()=default;
            /**
             * @brief Construct a new Differential Ik object
             * 
             * @param model 
             * @param data 
             * @param collisionModel 
             * @param collisionData 
             * @param ikOptions 
             */
            DifferentialIk(pin::Model& model, 
                    pin::Data& data, 
                    pin::GeometryModel& collisionModel,
                    pin::GeometryData& collisionData,
                    DifferentialIkOptions& ikOptions);
            
            /**
             * @brief Construct a new Differential Ik object
             * 
             * @param model 
             * @param collisionModel 
             * @param ikOptions 
             * @param qRef 
             */
            DifferentialIk(pin::Model& model, 
                        pin::GeometryModel& collisionModel,
                        DifferentialIkOptions& ikOptions,
                        rplState& qRef);
            /**
             * @brief Solve the inverse Kinematics problem.
             * 
             * @param targetFrame 
             * @param targetPose 
             * @param initSolutionJoint 
             * @return true : If able to solve the ik problem provided the solutions.
             * @return false 
             */
            bool solve(const std::string& targetFrame, 
            const pin::SE3& targetPose, 
            const std::optional<rplState>& initSolutionJoint = std::nullopt );
            
            /**
             * @brief Get the Result of Ik problem.
             * 
             * @return pin::Model::ConfigVectorType 
             */
            rplState getResult();
    
            private:
                void init();
            
                bool checkTeminationCondition(rplState& guessJointVector);
                
            private:
               pin::Model mPinModel;
                pin::Data mData;
                pin::GeometryModel mCollisionModel;
                pin::GeometryData mCollisionData;
                cpproboplan::inverseKinematics::DifferentialIkOptions mIkOptions;

                bool mIsSuccess;
                pin::Model::ConfigVectorType mSolutionJoint;
                cpproboplan::crRandVecGenerator<double> mRandomVecGenerator;

    };

}