/**
 * @file DifferentialIk.cpp
 * @author Prajwal Thakur
 * @brief  source file for DifferentialIk planner.
 *
 * */
#include "DifferentialIk.hpp"

namespace cpproboplan::inverseKinematics
{


    DifferentialIk::DifferentialIk()=default;
    
    /////////////////////////////////////////////////////////////////////////////////////////

    DifferentialIk::~DifferentialIk()=default;

    ///////////////////////////////////////////////////////////////////////////////////////

    DifferentialIk::DifferentialIk(pin::Model& model, 
                pin::Data& data, 
                pin::GeometryModel& collisionModel,
                pin::GeometryData& collisionData,
                DifferentialIkOptions& ikOptions):
                mPinModel(model),
                mData(data),
                mCollisionModel(collisionModel),
                mCollisionData(collisionData),
                mIkOptions(ikOptions)
                {
                    
                    init();
                }

    ///////////////////////////////// 
    //get a model  build the reduced model
    // collision model can be of the original model for the obstacle checking
    //TODO: better options ?
    DifferentialIk::DifferentialIk(pin::Model& model, 
                pin::GeometryModel& collisionModel,
                DifferentialIkOptions& ikOptions,
                pin::Model::ConfigVectorType& qRef):
                mIkOptions(ikOptions)
                {

                    // build the reduced model of the pinocchio after fxing the the ingored_joints at the neutral positions
                    pin::buildReducedModel(model,collisionModel,mIkOptions.ignore_joint_indices,qRef,mPinModel,mCollisionModel); 
                    mData = pin::Data(mPinModel);
                    mCollisionData = pin::GeometryData(mCollisionModel);
                    init();
                }
    /////////////////////////////////////////////////////////////////////////////////

    void DifferentialIk::init()
    {

        //create Random number specifically for some indices --> reduced model has taken care of this.
        std::vector<std::size_t> seedVector = cpproboplan::generateRandomSeed(mIkOptions.rng_seed, mPinModel.nv);
        mRandomVecGenerator = cpproboplan::crCreateRandVecGenerator(mIkOptions.distribution_type,seedVector,mPinModel, mIkOptions.joint_limit_padding);
        mIsSuccess = false;
        mSolutionJoint.setZero();


    }

    /////////////////////////////////////////////////////////////

    bool DifferentialIk::solve(const std::string& targetFrame, 
                        const pin::SE3& targetPose, 
                        const std::optional<pin::Model::ConfigVectorType>& initSolutionJoint )
    {
        init();
        

        // get the target frame id 
        std::size_t targetFrameId = mPinModel.getFrameId(targetFrame);

        std::size_t numActiveJoints = mPinModel.nv;
        //std::cerr << " num active joints " << mPinModel.nv << " " << mIkOptions.ignore_joint_indices.size() << " " <<  numActiveJoints << std::endl;
        // weights on joints for LM weighted IK
        Eigen::DiagonalMatrix<double,Eigen::Dynamic> weightJoint(numActiveJoints); 
        if(mIkOptions.joint_weights == std::nullopt)
        {
            weightJoint.diagonal() = Eigen::VectorXd::Ones(numActiveJoints);
        }
        else if (mIkOptions.joint_weights->size() != numActiveJoints || cpproboplan::hasNonpositive(*mIkOptions.joint_weights)==true)
        {
            std::cerr << "size doesnt match or one of the weights is less than or equal to 0" << std::endl;
        }
        else
        {
            weightJoint.diagonal() = Eigen::VectorXd::Map(mIkOptions.joint_weights->data(),mIkOptions.joint_weights->size());
        }
        
        weightJoint.diagonal() = weightJoint.diagonal().cwiseInverse();
        
        
        // if the guess Solution is not provided, we will generate one from  the reandom vector generator

        pin::Model::ConfigVectorType guessJointVector;
        if(!initSolutionJoint)
        {
            


            std::pair<pin::Model::ConfigVectorType,bool> res =  crGetRandomCollisionFreeSample(mPinModel, 
                                                                mData, 
                                                                mCollisionModel, 
                                                                mCollisionData, mIkOptions.collision_safety_margin, mIkOptions.max_tries, mRandomVecGenerator);
            if(!res.second)
            {
                std::cerr << " Not able to generate the collision free Joint Sample"<<std::endl;
                return false;
            }
            guessJointVector = res.first;
        
        }
        else
        {   
            // also remove the ignored_joint_indices , in this case last 2
            // todo: better way of handling ignored joint indices 
            guessJointVector = initSolutionJoint->head(numActiveJoints);
        }

        // make the current joint vector to be equal to the guess joint vector

        pin::Model::ConfigVectorType currJointVector = guessJointVector; 
        //double initErrorNorm  = std::numeric_limits<double>::max();
        Eigen::VectorXd v(numActiveJoints);
        typedef Eigen::Matrix<double,6,1> Vector6d;
        Vector6d err; // error in the end effector frame
        typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6xd;
        Matrix6xd J(6, numActiveJoints);
        Matrix6xd Jtemp(6,numActiveJoints);
        J.setZero();
        Jtemp.setZero();

        pin::Data::Matrix6 Jlog, JJt;
        // following algorithm  listed in section 6.2.2 : Modern Robotics, Kyvin M Lynch and reference #3
        std::size_t numReTries = 0, numIters = 0;
        
        std::cerr << "before while" << std::endl;
        while(numReTries < mIkOptions.max_retries)
        {   
            numIters = 0;
            while(numIters < mIkOptions.max_iters)
            {

                // forward kinematics to initial guess
                pin::forwardKinematics(mPinModel,mData,currJointVector);
                pin::updateFramePlacements(mPinModel, mData);
                // find the SE(3) error between current EE frame to  the desired EE frame in the EE frame
                //std::cerr << "before mData.oMf" << std::endl;
                const pin::SE3 iMd = mData.oMf[targetFrameId].actInv(targetPose);
                //std::cerr << "before err" << std::endl;

                err = pin::log6(iMd).toVector(); // get the twist vector in targetFrame frame ( if speed is zero then reached to gaol ?)
                
                std::cerr << " error in "<< numIters << " : " <<  err.transpose() << std::endl; //<< " current frame pose " << mData.oMf[targetFrameId] << " joint "<< mData.oMi[targetFrameId] << std::endl;
                
                if(err.head(3).norm() < mIkOptions.max_translation_error && err.tail(3).norm() < mIkOptions.max_rotation_error)
                {
                    
                    cpproboplan::crWrapToPi(currJointVector);
                    // cehck termination Condition
                    // true if it satisfy the termination condition otherwise false
                    if(!checkTeminationCondition(currJointVector)){break;}
                    mIsSuccess = true;
                    break;
                }
                // if the error is not less than the norm, take gradient step
                pin::computeFrameJacobian(mPinModel,mData,currJointVector,targetFrameId,pin::ReferenceFrame::LOCAL,J);
                // for(size_t i=0;i<numActiveJoints;++i)
                // {
                //     J.col(i) = Jtemp.col(activeJointIndices[i]);
                // }

                // Jacobian on manifold : see reference 3
                pin::Jlog6(iMd.inverse(),Jlog); // 6x6
                J  = - Jlog*J;   // 6x6.6x7=> 6x7
                JJt = J*J.transpose(); //6x6
                JJt.diagonal().array() += mIkOptions.damping; //6x6
                //std::cerr << "JJt size "<< JJt.rows() << " cols " << JJt.cols() << std::endl;

                // The nonalias method on the assignment operator is faster than a simple assignment operator,
                // provided there is no memory overlap between the left-hand side and the right-hand side.
                // For a linear system Ax = b, one can use A.solve(b). However, for faster computation,
                // A can first be decomposed, depending on the type of matrix A.
                // Reference: https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialLinearAlgebra.html
                v.noalias() = -J.transpose() * (JJt.ldlt().solve(err));
                
                //std::cerr << "joint velocity "<< v.size() << std::endl;

                // V is twist vector ( i.e spped in SE(3) space, integrate it to get the Joint Vector)
                currJointVector = pin::integrate(mPinModel,currJointVector,v*mIkOptions.deltaTime);
                
                numIters++;
            }

            numReTries++;
            if(mIsSuccess)
            {
                std::cerr << " solved in "<< numReTries << " tries "<<std::endl;
                mSolutionJoint = currJointVector;
                break;
            }
            else
            {
                std::pair<pin::Model::ConfigVectorType,bool> res =  crGetRandomCollisionFreeSample(mPinModel, 
                                                                    mData, 
                                                                    mCollisionModel, 
                                                                    mCollisionData, mIkOptions.collision_safety_margin, mIkOptions.max_tries, mRandomVecGenerator);
                if(!res.second)
                {
                    std::cerr << " coulldnt able to generate the collision free Joint Sample"<<std::endl;
                    break;
                }
                currJointVector = res.first;

            }  
        }

        //TODO: better way of handling ignored joint vectors
        mSolutionJoint.resize(currJointVector.size() + 2);
        mSolutionJoint.head(currJointVector.size()) = currJointVector;
        mSolutionJoint.tail(2) = initSolutionJoint->tail(2);
        return mIsSuccess;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////

    bool DifferentialIk::checkTeminationCondition( pin::Model::ConfigVectorType& guessJointVector)
    {

        bool isJointInLimit  = cpproboplan::crCheckWithinLimit(mPinModel,guessJointVector);
        if(!isJointInLimit)
        {
            crClamp(mPinModel,guessJointVector);
            std::cerr <<" voilating Joint Limits : clamped";
        }
        double distancePadding = 0.0;
        bool isColliding = cpproboplan::crCheckCollision(mPinModel, 
                            mCollisionModel, 
                            guessJointVector, 
                            mData,
                            mCollisionData,
                            distancePadding,
                            true
                        );
        if(isColliding)
        {
            std::cerr<<" In joint limit but in collision";
            return false;
        }
        std::cerr<<"Ik optimization solved, within limit and collision free";
        return true;
    }

    //////////////////////////////////

    pin::Model::ConfigVectorType DifferentialIk::getResult()
    {
        return mSolutionJoint;
    }

    

}