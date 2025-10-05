/**
 * @file Memory.hpp
 * @author Prajwal Thakur (you@domain.com)
 * @brief  header file containing definations for typedefs.
 * */


#pragma once
#include <Eigen/Dense>
#include <vector>
#include <optional>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <vector>
#include <unordered_set>
#include <memory>

//typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;
//typedef VectorXs ConfigVectorType;
using rplState = pinocchio::Model::ConfigVectorType;

using rplspatialPose =  pinocchio::SE3Tpl<pinocchio::Model::Scalar, pinocchio::Model::Options>;
// Collection of poses (per-object), matching Pinocchio's style:
using rplspatialPoseCollection = std::vector<rplspatialPose>;

using rplGeomIndex = std::size_t;

using rplNodeIndex = std::size_t ; 

using rplNodeName = std::string;

using rplUnSignedInt = std::size_t;

template <typename T>
using rplStlCollection = std::vector<T>;

template <typename T>
using rplStl2dCollection = std::vector<std::vector<T>>;

template <typename T>
using rplCollection = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template <typename T> 
using rPlUnorderedSet = std::unordered_set<T>;

template <typename T, typename Q>
using rPlUnorderedMap = std::unordered_map<T,Q>;

template <typename T>
using rplUniquePtr = std::unique_ptr<T>;

template <typename T>
using rplSharedPtr = std::shared_ptr<T>;

template <typename T>
using rplwkPtr = std::weak_ptr<T>;

template <typename T>
using rplOpt = std::optional<T>;



