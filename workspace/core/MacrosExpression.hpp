#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <string>
#include "Memory.hpp"

namespace cpproboplan
{

inline void checkQSize(const pinocchio::Model &model, const cpproboplan::rplqState &q) {
    assert(q.rows() == model.nq && "q size mismatch with model.nq");
}

inline std::string getNodeName(const rplNodeIndex& rplNodeIndex)
{
    std::string nodeName = "N" + std::to_string(rplNodeIndex);
    return nodeName;
}



}
