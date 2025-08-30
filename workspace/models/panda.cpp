#include "panda.hpp"
namespace pin = pinocchio;
namespace scenario::panda
{

    /**
     * @brief Add visualization and collision object
     * 
     */
    void addObject(std::string nameObj,
        int parentFrameId,
        Eigen::Matrix3d rot,
        Eigen::Vector3d translation,
        Eigen::Vector4d color,
        std::shared_ptr<coal::ShapeBase> Collgeometry,
        pin::GeometryModel& visualModel,
        pin::GeometryModel& collisionModel
        )
    {


        // add the plane
        pin::SE3 visPlacement(rot, translation);
        pin::GeometryObject obj(
            nameObj,
            parentFrameId,
            Collgeometry,
            visPlacement
        );
        obj.meshColor = color;
        visualModel.addGeometryObject(obj);
        collisionModel.addGeometryObject(obj);

    }

    /**
     * @brief Adds obstacles and collisions to the Panda collision model.
     * 
     * @param model `pinocchio.Model`
     * @param collisionModel  The Panda collision geometry model.
     * @param visualModel  The Panda visual geometry model.
     * @param inflationRadius  optional An inflation radius, in meters, around the objects.
     */
    void addObjectCollisions(pin::Model& model, 
        pin::GeometryModel& collisionModel,
        pin::GeometryModel& visualModel,double inflationRadius)
        {
            
        std::shared_ptr<coal::ShapeBase> Collgeometry ;  
        // add the plane
        Collgeometry = std::make_shared<coal::Box>(2.0, 2.0, 0.3);
        addObject("ground_plane",0,Eigen::Matrix3d::Identity(),Eigen::Vector3d(0.0,0.0,-0.151),Eigen::Vector4d(0.5,0.5,0.5,0.5),Collgeometry,visualModel,collisionModel);

        // add the sphere1

        Collgeometry = std::make_shared<coal::Sphere>(0.2 + inflationRadius);
        addObject("obstacle_sphere_1",0,
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(0.0,0.1,0.8),
            Eigen::Vector4d(0.0, 1.0, 0.0, 0.5),Collgeometry,visualModel,collisionModel);

        // // add sphere2
        Collgeometry = std::make_shared<coal::Sphere>(0.25 + inflationRadius);
        addObject("obstacle_sphere_2",0,
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(0.15, -1.0, 0.5),
            Eigen::Vector4d(1.0, 1.0, 1.0, 0.5),Collgeometry,visualModel,collisionModel);

        // add sphere3 
        Collgeometry = std::make_shared<coal::Sphere>(0.07 + inflationRadius);
        addObject("obstacle_sphere_3",0,
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(0.2,0.1,0.15),
            Eigen::Vector4d(1.0, 1.0, 0.0, 0.5),Collgeometry,visualModel,collisionModel);
        
        // add sphere4
        Collgeometry = std::make_shared<coal::Sphere>(0.07 + inflationRadius);
        addObject("obstacle_sphere_4",0,
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(-0.5,0.1,0.4),
            Eigen::Vector4d(0.0, 0.0, 1.0, 0.5),Collgeometry,visualModel,collisionModel);        
        
        // add box1
        Collgeometry = std::make_shared<coal::Box>( 0.25 + 2.0 * inflationRadius,
                                                    0.55 + 2.0 * inflationRadius,
                                                    0.55 + 2.0 * inflationRadius);
        addObject("obstacle_box_1",0,
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(-0.8, 0.5, 0.7),
            Eigen::Vector4d(1.0, 0.0, 0.0, 0.5),Collgeometry,visualModel,collisionModel);



        // add box2
        Collgeometry = std::make_shared<coal::Box>( 0.33 + 2.0 * inflationRadius,
                                                    0.33 + 2.0 * inflationRadius,
                                                    0.33 + 2.0 * inflationRadius);
        addObject("obstacle_box_2",0,
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d(-0.8, -0.5, 0.75),
            Eigen::Vector4d(0.0, 0.0, 1.0, 0.5),Collgeometry,visualModel,collisionModel);

        //Define the active collision pairs between the robot and obstacle links.
        std::vector<std::string> collisionNames;
        for(const auto& cObj : collisionModel.geometryObjects)
        {  
            // to filter out the collision model related to panda only, find panda in name
            if(cObj.name.find("panda") != std::string::npos)
            {
                collisionNames.push_back(cObj.name);
            }
        }
        std::vector<std::string> obstacleName = { "ground_plane",
                                                "obstacle_box_1",
                                                "obstacle_box_2",
                                                "obstacle_sphere_1",
                                                "obstacle_sphere_2",
                                                "obstacle_sphere_3",
                                                "obstacle_sphere_4"
                                                };
        // now add the collision between newly created objs and all the panda links.
        for(auto& cobj: collisionNames)
        {
            for(auto& obj:obstacleName)
            {
            cpproboplan::crSetCollisions(model,collisionModel,cobj,obj,true);

            }
        }

        // Exclude collisions between ground and base link
        cpproboplan::crSetCollisions(model,collisionModel, "panda_link0", "ground_plane", false);
        cpproboplan::crSetCollisions(model,collisionModel, "ground_plane", "obstacle_sphere_3", false);

        }


        void addSelfCollisions(pin::Model& model, pin::GeometryModel& collisionModel, std::string srdfFileName)
        {

            collisionModel.addAllCollisionPairs();
            pinocchio::srdf::removeCollisionPairs(model,collisionModel,srdfFileName);

        }
    
}
