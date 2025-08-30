#pragma once
#include <pinocchio/multibody/geometry.hpp>
#include "pinocchio/parsers/srdf.hpp"
#include <pinocchio/spatial/se3.hpp>
#include <coal/shape/geometric_shapes.h>
#include "/root/workspace/src/core/core.h"

namespace pin = pinocchio;
namespace scenario::panda
{

    void addObject(std::string nameObj,
        int parentFrameId,
        Eigen::Matrix3d rot,
        Eigen::Vector3d translation,
        Eigen::Vector4d color,
        std::shared_ptr<coal::ShapeBase> Collgeometry,
        pin::GeometryModel& visualModel,
        pin::GeometryModel& collisionModel
        );

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
        pin::GeometryModel& visualModel,double inflationRadius);


    void addSelfCollisions(pin::Model& model, pin::GeometryModel& collisionModel, std::string srdfFilename);



}
