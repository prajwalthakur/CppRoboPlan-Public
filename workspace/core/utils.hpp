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
        rplState& q,
        pin::Data & data,
        pin::GeometryData & collisionData,
        const double distancePadding,
        const bool stopAtFirstCollision = true); // For faster computation.


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
        rplState& q,
        const double distancePadding);


    /**
    Checks whether a path consisting of multiple joint configurations is collision-free.
    */

    bool crIfCollFreePath(pin::Model & model,
        pin::GeometryModel& collisionModel,
        rplStlCollection<rplState>& q_path,
        pin::Data& data,
        pin::GeometryData& collisionData,
        const double distancePadding,
        const bool stopAtFirstCollision = true);


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
    rplStlCollection<int> getCollisionGeometryIds(pin::Model & model,
    pin::GeometryModel& collisionModel,
    const std::string& body);



    /**
     * @brief Sets collision checking between two bodies by searching for their corresponding geometry objects in the collision model.
     * 
     * @param model The model to use for getting frame IDs.
     * @param collisionModel The model to use for collision checking.
     * @param body1 The name of the first body.
     * @param body2 The name of the second body.
     * @param enable If True, enables collisions. If False, disables collisions.
     */
    void crSetCollisions(pin::Model & model,
    pin::GeometryModel& collisionModel,
    const std::string& body1, const std::string& body2, const bool enable);
    /**
     * @brief
    */
    double crConfigurationDistance(rplState& q1, rplState& q2 );
    /**
     * @brief
    */
    double crGetPathLength(rplStlCollection<rplState>& q_path);
    /**
     * @brief
    */
    rplspatialPose  crGetCartesianPose(pin::Model& model, 
        const std::string& targetFrame,  
        rplState& qVec, 
        pin::Data& data);
    /**
     * @brief
    */
    rplspatialPoseCollection  crGetCartesianPoses(pin::Model& model, 
    const std::string& targetFrame,  
    rplStlCollection<rplState>& qVec, 
    pin::Data& data);
    /**
     * @brief
    */
    rplState crGetRandomSample(crRandVecGenerator<double>&);
    /**
     * @brief Get a sample which is collision free.
    */
    std::pair<rplState,bool> crGetRandomCollisionFreeSample(pin::Model& model, 
        pin::Data& data, 
        pin::GeometryModel& collisionModel, 
        pin::GeometryData& collisionData, 
        const double distancePadding, 
        const rplUnSignedInt maxTries,
        crRandVecGenerator<double>& randomVecGenerator);
    /**
     * @brief Wrap the each entries to [-pi,pi]
    */

    void crWrapToPi(rplState& q);
    /**
     * @brief Check if the the Joint-State are in limits
    */
    bool crCheckWithinLimit(pin::Model& model, rplState& q);
    /**
     * @brief Clamp the Joint vector 
    */
    void crClamp(pin::Model& model, rplState& q);
    /**
     * @brief Generate a  random number generator from a given seed and distribution.
    */    
    crRandomGenerator<double> crCreateRandomGenerator(const 
    std::string& distType, 
    const rplUnSignedInt, 
    const double minRange, 
    const double maxRange);
    /**
     * @brief Generate a  random Vector number generator from a given seed and distribution.
    */
    crRandVecGenerator<double> crCreateRandVecGenerator(const std::string& distributionType,
        const rplUnSignedInt dim, 
        const rplCollection<rplUnSignedInt>& seed, 
        const rplCollection<double>& minRange, 
        const rplCollection<double>& maxRange,
        const double padding=0.0);
    /**
     * @brief Generate a  random Vector number generator to generate the random states whose limit is set by model's Joint States.
    */
    crRandVecGenerator<double> crCreateRandVecGenerator(const std::string& distributionType, 
    const rplCollection<rplUnSignedInt>& seed, 
    const pin::Model& model, const double padding = 0.0);

    /**
     * @brief Generate a  random seed.
    */
    rplCollection<rplUnSignedInt> generateRandomSeed(rplUnSignedInt rng_seed, rplUnSignedInt dim);

    /**
     * @brief Function to wait for a keypress.
    */
    char getKeyPress();
    /**
     * @brief Function to wait for a particular keypress.
    */
    void crWaitForKeyPress(char triggerKey = '\n');
    /**
     * @brief check if any element in vector has any element equal or less than 0
    */
    bool hasNonpositive(const rplStlCollection<double>& vec);
    bool hasNonpositive(const rplCollection<double>& vec);
}
