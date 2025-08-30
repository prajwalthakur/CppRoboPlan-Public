#include "VisJointTrajectory.hpp"

namespace cpproboplan::visualizer
{

    visJointTrajectory::visJointTrajectory()=default;
    
    visJointTrajectory::~visJointTrajectory()=default;

    ///////////////////////////////

    visJointTrajectory::visJointTrajectory(
                const pin::Model& model,
                const pin::GeometryModel& visualModel,
                const pin::GeometryModel& collisionModel,
                const VisualizerOptions& visualizerOptions):
                mVisualizerOptions(visualizerOptions),
                mPinModel(model),
                mData(mPinModel),
                mVisualModel(visualModel),
                mVisualData(mVisualModel),
                mCollisionModel(collisionModel),
                mCollisionData(mCollisionModel),
                mMeshCatPtr(std::make_shared<Meshcat>(7005))
    {
        init();
    }

    ////////////////////////////
    
    void visJointTrajectory::addCoalBoxToMeshacat(const std::shared_ptr<coal::Box>& box, const pin::GeometryModel::GeometryObject& go )
    {
        mMeshCatPtr->SetObject("environment/" + go.name,

                        drake::geometry::Box(box->halfSide[0]*2,
                            box->halfSide[1]*2,
                            box->halfSide[2]*2),
                        drake::geometry::Rgba(go.meshColor[0],
                            go.meshColor[1],
                            go.meshColor[2],
                            go.meshColor[3]));
        Eigen::Quaterniond rot(go.placement.rotation());
        drake::math::RigidTransform<double> XPose
        (
            rot, 
            go.placement.translation()
        );
        mMeshCatPtr->SetTransform("environment/" + go.name, XPose);
    }
    
    ////////////////////////////

    void visJointTrajectory::addCoalSphereToMeshCat(const std::shared_ptr<coal::Sphere>& sphere, const pin::GeometryModel::GeometryObject& go)
    {
        mMeshCatPtr->SetObject("environment/" + go.name,
                                drake::geometry::Sphere(sphere->radius),
                                drake::geometry::Rgba(go.meshColor[0],
                                    go.meshColor[1],
                                    go.meshColor[2],
                                    go.meshColor[3]));
        Eigen::Quaterniond rot(go.placement.rotation());
        drake::math::RigidTransform<double> XPose
        (
            rot, 
            go.placement.translation()
        );
        mMeshCatPtr->SetTransform("environment/" + go.name, XPose);
    }
    
    /////////////////////////

    void visJointTrajectory::init()
    {
        mVisualRate = mVisualizerOptions.visualizer_rate;
        mSleepDuration = 1.0/mVisualRate;
        std::cerr << "Open in browser: " << mMeshCatPtr->web_url() << std::endl; // handy
        
        // ----- Upload meshes to Meshcat once -----
        for (const auto& go : mVisualModel.geometryObjects) {
            if (!go.meshPath.empty()) {
            // Pinocchio stores an absolute mesh path + scale for mesh geometries.
            // (If meshScale is non-uniform, we’ll use the x scale as an approximation.)
            double s = go.meshScale[0] > 0 ? go.meshScale[0] : 1.0;
            mMeshCatPtr->SetObject("robot/" + go.name, Mesh(go.meshPath, s));
            //std::cerr<<"meshcat path not emplty "<< go.name << std::endl;
            }
            else if( go.geometry )
            {
                    if (auto box = std::dynamic_pointer_cast<coal::Box>(go.geometry))
                    {
                        addCoalBoxToMeshacat(box, go);
                        std::cerr<<"adding box "<< go.name << std::endl;
                    }
                    else if ( auto sphere = std::dynamic_pointer_cast<coal::Sphere>(go.geometry))
                    {
                        addCoalSphereToMeshCat(sphere, go);
                        std::cerr<<"adding sphere "<< go.name << std::endl;

                    }
            }
        }

        mEEPath.clear();
    }

    std::shared_ptr<Meshcat> visJointTrajectory::getMeshcatPtr()
    {
        return mMeshCatPtr;
    }
    
    ///////////////////////////////////////////////////

    void visJointTrajectory::setStartTime(const std::chrono::high_resolution_clock::time_point currTime)
    {
        mStartTime = std::chrono::duration<double>(currTime.time_since_epoch()).count();
    }

    double visJointTrajectory::elapsedTime(const std::chrono::high_resolution_clock::time_point currTime)
    {
        return std::chrono::duration<double>(currTime.time_since_epoch()).count() - mStartTime;
    }

    ///////////////////////////////////////////////////

    void visJointTrajectory::updateMeshcatTransforms() 
    {

        for (std::size_t i = 0; i < mVisualModel.geometryObjects.size(); ++i) 
        {
            const auto& go = mVisualModel.geometryObjects[i];
            const auto& M  = mVisualData.oMg[i];
            drake::math::RotationMatrix<double> R_WG(M.rotation());
            drake::math::RigidTransform<double>  X_WG(R_WG, M.translation());
            mMeshCatPtr->SetTransform("robot/" + go.name, X_WG);
        }
    }

    /////////////////////////////////////////////////////////

    void visJointTrajectory::setStartJointPose(pin::Model::ConfigVectorType JointState)
    {
        pin::forwardKinematics(mPinModel, mData, JointState);
        pin::updateFramePlacements(mPinModel, mData);
        pin::updateGeometryPlacements(mPinModel, mData, mVisualModel, mVisualData);        
        updateMeshcatTransforms();
    }

    ///////////////////////////////////////////////////////

    void visJointTrajectory::showStartAndGoalEEPose(pin::Model::ConfigVectorType startJointPose, pin::Model::ConfigVectorType goalJointPose)
    {   
        int eeId = mPinModel.getFrameId(mVisualizerOptions.ee_frame_name);
        
        pin::forwardKinematics(mPinModel,mData,startJointPose);
        pin::updateFramePlacements(mPinModel, mData);
        double radius =   mVisualizerOptions.visRadius;
        const auto& MStart =  mData.oMf[eeId];
        drake::math::RigidTransform<double> X_WStart
        (
            drake::math::RotationMatrix<double>(MStart.rotation()), 
            MStart.translation()
        );
        // Red sphere for start
        // mMeshCatPtr->SetObject("markers/start_ee", 
        //                     drake::geometry::Sphere(radius),
        //                     drake::geometry::Rgba(0.643,0,1,1));
        // mMeshCatPtr->SetTransform("markers/start_ee", X_WStart);
        AddFrameAxes( "markers/start_axes", X_WStart,
                    /*axis_length=*/radius*3.0, /*axis_radius=*/radius*0.15);






        //std::cerr << "Start Frame " << eeId << " placement:\n" << MStart << std::endl;
        
        
        // Compute forward kinematics for goal pose
        pin::forwardKinematics(mPinModel, mData, goalJointPose);
        pin::updateFramePlacements(mPinModel, mData);
        const auto& M_goal = mData.oMf[eeId];

        drake::math::RigidTransform<double> X_WGoal
        (
            drake::math::RotationMatrix<double>(M_goal.rotation()), 
            M_goal.translation()
        );      

        //std::cerr << "Goal Frame " << eeId << " placement:\n" << M_goal << std::endl;
        // Green sphere for goal
        // mMeshCatPtr->SetObject("markers/goal_ee", 
        //                     drake::geometry::Sphere(radius),
        //                     drake::geometry::Rgba(0.647,0.165,0.165,1));
        // mMeshCatPtr->SetTransform("markers/goal_ee", X_WGoal);
        AddFrameAxes("markers/goal_axes", X_WGoal,
                    /*axis_length=*/radius*3.0, /*axis_radius=*/radius*0.15);

        return;
    }

    
    //////////////////////////////////////////////////////////

    void visJointTrajectory::updateEEPath()
    {
        int ee_id = mPinModel.getFrameId(mVisualizerOptions.ee_frame_name);
        const auto& M_ee = mData.oMf[ee_id];
        mEEPath.push_back(M_ee.translation());

        if (mEEPath.size() > 1) {
            Eigen::MatrixXd pts(3, mEEPath.size());
            for (size_t i = 0; i < mEEPath.size(); ++i)
                pts.col(i) = mEEPath[i];

            mMeshCatPtr->SetLine("markers/ee_path", pts,
                                2.0,   // line thickness
                                drake::geometry::Rgba(0, 0, 1, 1));  // blue
        }
    }

    //////////////////////////////////////////////////////////////

    void visJointTrajectory::stepSim(pin::Model::ConfigVectorType JointState)
    {

        pin::forwardKinematics(mPinModel, mData, JointState);
        pin::updateGeometryPlacements(mPinModel, mData, mVisualModel, mVisualData);        
        updateMeshcatTransforms() ;
        updateEEPath()  ;
        std::this_thread::sleep_for(std::chrono::duration<double>(mSleepDuration));


    }

    ////////////////////////////////////////////////////////////

    void visJointTrajectory::stepSim(pin::Model::ConfigVectorType JointState, pin::Model::TangentVectorType JointSpeed)
    {

        pin::forwardKinematics(mPinModel, mData, JointState,JointSpeed);
        pin::updateGeometryPlacements(mPinModel, mData, mVisualModel, mVisualData);
        updateMeshcatTransforms() ;
        updateEEPath() ;
        std::this_thread::sleep_for(std::chrono::duration<double>(mSleepDuration));


    }

    ///////////////////////////////////////////////////////////////////////

    void visJointTrajectory::stepSim(pin::Model::ConfigVectorType JointState, pin::Model::TangentVectorType JointSpeed, pin::Model::TangentVectorType Jointacc)
    {

        pin::forwardKinematics(mPinModel, mData, JointState,JointSpeed,Jointacc);
        pin::updateGeometryPlacements(mPinModel, mData, mVisualModel, mVisualData);        
        updateMeshcatTransforms();
        updateEEPath() ;
        std::this_thread::sleep_for(std::chrono::duration<double>(mSleepDuration));


    }

    ///////////////////////////////////////////////////////////////////////////////////

    void visJointTrajectory::AddFrameAxes(
                    const std::string& path,
                    const drake::math::RigidTransform<double>& X_WF,
                    double axis_length ,
                    double axis_radius ) 
    {
        using drake::geometry::Cylinder;
        using drake::geometry::Rgba;
        const Eigen::Matrix3d R_WF = X_WF.rotation().matrix();
        const Eigen::Vector3d p_WF = X_WF.translation();

        // X-axis: the cylinder’s +Z  is aligned with the EE’s +X.
        {
            Eigen::Matrix3d R_WC = R_WF*Eigen::AngleAxisd(+M_PI/2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();;
            Eigen::Vector3d p_WC = p_WF + R_WF * (Eigen::Vector3d::UnitX() * (axis_length/2.0));
            drake::math::RigidTransform<double> X_(drake::math::RotationMatrix<double>(R_WC), p_WC);   
            mMeshCatPtr->SetObject(path + "/x_axis", Cylinder(axis_radius, axis_length), Rgba(1,0,0,1));
            mMeshCatPtr->SetTransform(path + "/x_axis", X_);
        }

        // Y-axis: the cylinder’s +Z  is aligned with the EE’s +Y.
        {
            Eigen::Matrix3d R_WC = R_WF *
                Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
            Eigen::Vector3d p_WC = p_WF + R_WF * (Eigen::Vector3d::UnitY() * (axis_length/2.0));
            drake::math::RigidTransform<double> Y_(drake::math::RotationMatrix<double>(R_WC), p_WC); 
            mMeshCatPtr->SetObject(path + "/y_axis", Cylinder(axis_radius, axis_length), Rgba(0,1,0,1)); 
            mMeshCatPtr->SetTransform(path + "/y_axis", Y_);
        }

        // Z-axis: cylinder already along local +Z
        {
            Eigen::Matrix3d R_WC = R_WF;
            Eigen::Vector3d p_WC = p_WF + R_WF * (Eigen::Vector3d::UnitZ() * (axis_length/2.0));
            drake::math::RigidTransform<double> Z_(drake::math::RotationMatrix<double>(R_WC), p_WC); 
            mMeshCatPtr->SetObject(path + "/z_axis", Cylinder(axis_radius, axis_length), Rgba(0,0,1,1));
            mMeshCatPtr->SetTransform(path + "/z_axis", Z_);
        }
    }


}