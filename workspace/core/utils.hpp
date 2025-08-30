/**
 * @file utils.hpp
 * @author Prajwal Thakur (you@domain.com)
 * @brief  A collection of utility functions and helpers used across the project
 * */
#pragma once
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>   // updateGeometryPlacements
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/collision.hpp>
#include "RandomVec.hpp"
#include "Random.hpp"
#include "Memory.hpp"
#include "MacrosExpression.hpp"
namespace pin = pinocchio;
namespace cpproboplan
{  

    /**
    * @brief Checks whether a specified joint configuration is collision-free.
    *
    * Computes the forward kinematics, updates the geometry placements, and
    * calls `computeCollision` for every active pair in the GeometryData.
    *
    * @tparam ConfigVectorType Type of the joint configuration vector.
    *
    * @param[in] model Robot model (const).
    * @param[in] collisionModel Collision geometry model (const).
    * @param[in] q Robot configuration vector.
    * @param[in,out] data Corresponding data (non-const) where forward kinematics results are stored.
    * @param[in,out] collisionData Corresponding geometry data (non-const) where collision results are computed.
    * @param[in] distancePadding Padding, in meters, to apply to the minimum allowed collision distance.
    * @param[in] stopAtFirstCollision If true, stops checking after the first collision is detected.
    *
    * @return true If there are any collisions or minimum distance violations.
    * @return false Otherwise.
    *
    * @warning If `stopAtFirstCollision` is true, the collisions vector will not
    *          be completely filled.
    *
    * @note A similar function is available without model, data, and q,
    *       which does not recompute the forward kinematics.
    */
    bool crCheckCollision(pin::Model & model,
        pin::GeometryModel & collisionModel,
        pin::Model::ConfigVectorType & q,
        pin::Data & data,
        pin::GeometryData & collisionData,
        double& distancePadding,
        bool stopAtFirstCollision = true); // for faster computation


    /**
    * @brief Gets the minimum distance to collision at a specified state.
    *
    * Computes the forward kinematics, updates the geometry placements, and
    * calls `computeDistance` for every active pair in the GeometryData.
    *
    * @tparam ConfigVectorType Type of the joint configuration vector.
    *
    * @param[in] model Robot model (const).
    * @param[in,out] data Corresponding data (non-const) where FK results are stored.
    * @param[in] collisionModel Geometry model (const).
    * @param[in,out] collisionData Corresponding geometry data (non-const) where distances are computed.
    * @param[in] q Robot configuration vector.
    * @param[in] distancePadding The padding, in meters, to use for distance to nearest collision.
    *
    * @return The minimum distance to collision, in meters.
    *
    * @note A similar function is available without model, data, and q,
    *       which does not recompute the forward kinematics.
    */
    double crMinDistanceToCollision(pin::Model& model,
        pin::Data& data,
        pin::GeometryModel& collisionModel,
        pin::GeometryData& collisionData,
        pin::Model::ConfigVectorType& q,
        double distancePadding);


    /**
    Checks whether a path consisting of multiple joint configurations is collision-free.

    */
    bool crCheckCollisionAlongPath(pin::Model & model,
        pin::GeometryModel& collisionModel,
        std::vector<pin::Model::ConfigVectorType>& q_path,
        pin::Data& data,
        pin::GeometryData& collisionData,
        double distancePadding,
        bool stopAtFirstCollision = true);


    /**
     * @brief Gets a list of collision geometry model IDs for a specified body name.
     * 
     * @param model The model to use for getting frame IDs.
     * @param collisionModel  The model to use for collision checking
     * @param body  The name of the body.
                    This can be directly the name of a geometry in the collision model,
                    or it can be the name of the frame in the main model.
     * @return std::vector<int> 
     */
    std::vector<int> getCollisionGeometryIds(pin::Model & model,pin::GeometryModel& collisionModel,std::string body);



    /**
     * @brief Sets collision checking between two bodies by searching for their corresponding geometry objects in the collision model.
     * 
     * @param model The model to use for getting frame IDs.
     * @param collisionModel The model to use for collision checking.
     * @param body1 The name of the first body.
     * @param body2 The name of the second body.
     * @param enable If True, enables collisions. If False, disables collisions.
     */
    void crSetCollisions(pin::Model & model,pin::GeometryModel& collisionModel,std::string body1,std::string body2, bool enable);
   

    /**

    */
    double crConfigurationDistance(pin::Model::ConfigVectorType& q1, 
        pin::Model::ConfigVectorType& q2 );

    /**
        */
    double crGetPathLength(std::vector<pin::Model::ConfigVectorType>& );

    
    pin::Model::ConfigVectorType crGetRandomSample(crRandVecGenerator<double>&);


    std::pair<pin::Model::ConfigVectorType,bool> crGetRandomCollisionFreeSample(pin::Model& model, 
        pin::Data& data, 
        pin::GeometryModel& collisionModel, 
        pin::GeometryData& collisionData, 
        double& distancePadding, 
        std::size_t& maxTries,
        crRandVecGenerator<double>& randomVecGenerator);

    bool crCheckWithinLimit(pin::Model& model, pin::Model::ConfigVectorType& q);


    rplspatialPose  crGetCartesianPose(pin::Model& model, 
        std::string targetFrame,  
        rplqState& qVec, 
        pin::Data& data);


    rplspatialPoseCollection  crGetCartesianPoses(pin::Model& model, 
        std::string targetFrame,  
        rplqStateCollection& qVec, 
        pin::Data& data);

    crRandomGenerator<double> crCreateRandomGenerator(std::string, const std::size_t, const double minRange, const double maxRange);

    crRandVecGenerator<double> crCreateRandVecGenerator(std::string distributionType,
        const std::size_t dim, 
        const std::vector<std::size_t> seed, 
        const std::vector<double>& minRange, 
        const std::vector<double>& maxRange,double padding=0.0);

    crRandVecGenerator<double> crCreateRandVecGenerator(std::string distributionType, 
    const std::vector<std::size_t>seed, 
    const pin::Model& model, double padding=0.0);


    std::vector<std::size_t> generateRandomSeed(std::size_t rng_seed, std::size_t dim);



    // Function to wait for a keypress without Enter
    char getKeyPress();

    // check if any element in vector has any element equal or less than 0
    bool hasNonpositive(std::vector<double> vec);

    // wrap the each entries to [-pi,pi]
    void crWrapToPi(pin::Model::ConfigVectorType& q);


    // clamp the Joint vector 
    void crClamp(pin::Model& model, pin::Model::ConfigVectorType& q);
    
    void crWaitForKeyPress(char triggerKey = '\n');

}
