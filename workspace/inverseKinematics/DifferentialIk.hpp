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
            DifferentialIk();
        
        
            ~DifferentialIk();

            DifferentialIk(pin::Model& model, 
                    pin::Data& data, 
                    pin::GeometryModel& collisionModel,
                    pin::GeometryData& collisionData,
                    DifferentialIkOptions& ikOptions);
            
            
            DifferentialIk(pin::Model& model, 
                        pin::GeometryModel& collisionModel,
                        DifferentialIkOptions& ikOptions,
                        pin::Model::ConfigVectorType& qRef);
            
            bool solve(const std::string& targetFrame, 
            const pin::SE3& targetPose, 
            const std::optional<pin::Model::ConfigVectorType>& initSolutionJoint = std::nullopt );
            
           pin::Model::ConfigVectorType getResult();
            

            private:
                void init();
            
                bool checkTeminationCondition( pin::Model::ConfigVectorType& guessJointVector);
                
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