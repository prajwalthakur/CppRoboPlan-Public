#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>   // updateGeometryPlacements
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <drake/geometry/meshcat.h>
#include <drake/geometry/shape_specification.h>   // Mesh, Box, etc.
#include <drake/math/rigid_transform.h>
#include <drake/geometry/meshcat_params.h>

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <thread>

#ifndef PINOCCHIO_MODEL_DIR
#  define PINOCCHIO_MODEL_DIR "/root/workspace/src/models"
#endif
/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv) {
  namespace pin = pinocchio;
  using drake::geometry::Meshcat;
  using drake::geometry::Mesh;
  using drake::math::RigidTransformd;

  // ----- Load URDF into Pinocchio (model + visual geometry) -----
  const std::string urdf =
      (argc > 1) ? argv[1]
                 : std::string(PINOCCHIO_MODEL_DIR) +
                   "/panda_description/urdf/panda.urdf";

  std::vector<std::string> package_dirs;
  if (argc > 2) package_dirs.push_back(argv[2]);
  package_dirs.push_back(PINOCCHIO_MODEL_DIR);

  pin::Model model;
  pin::urdf::buildModel(urdf, model);
  pin::Data data(model);

  pin::GeometryModel vmodel;
  pin::urdf::buildGeom(model, urdf, pin::VISUAL, vmodel, package_dirs);

  pin::GeometryData vdata(vmodel);

  // ----- Start Meshcat server (opens local websocket/http) -----
  auto meshcat = std::make_shared<drake::geometry::Meshcat>(7005);
  // auto meshcat = std::make_shared<drake::geometry::Meshcat>("0.0.0.0", 7000);
  
  //auto meshcat = std::make_shared<Meshcat>();   // background server thread
  std::cout << "Open in browser: " << meshcat->web_url() << std::endl; // handy

  // ----- Upload meshes to Meshcat once -----
  for (const auto& go : vmodel.geometryObjects) {
    if (!go.meshPath.empty()) {
      // Pinocchio stores an absolute mesh path + scale for mesh geometries.
      // (If meshScale is non-uniform, we’ll use the x scale as an approximation.)
      double s = go.meshScale[0] > 0 ? go.meshScale[0] : 1.0;
      meshcat->SetObject("robot/" + go.name, Mesh(go.meshPath, s));
    }
    // (Primitives could be handled via drake::geometry::Box/Sphere/Cylinder similarly.)
  }

  // ----- Animate: FK, then push frame poses to Meshcat -----
 Eigen::VectorXd q = pin::neutral(model);
  for (int k = 0; k < 600; ++k) {
    if (model.nq > 0) q[0] = 0.6 * std::sin(0.01 * k);

    pin::forwardKinematics(model, data, q);
    pin::updateGeometryPlacements(model, data, vmodel, vdata);

    // Update each visual object’s pose
  for (std::size_t i = 0; i < vmodel.geometryObjects.size(); ++i) {
    const auto& go = vmodel.geometryObjects[i];
    const auto& M  = vdata.oMg[i];               // Pinocchio SE3 pose

    // Convert Eigen 3×3 to Drake's RotationMatrix
    drake::math::RotationMatrix<double> R_WG(M.rotation());
    drake::math::RigidTransform<double>  X_WG(R_WG, M.translation());

    meshcat->SetTransform("robot/" + go.name, X_WG);
  }
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  return 0;
}
