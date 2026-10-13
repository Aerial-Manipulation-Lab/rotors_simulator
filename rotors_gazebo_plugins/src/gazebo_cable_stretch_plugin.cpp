#include "rotors_gazebo_plugins/gazebo_cable_stretch_plugin.h"

#include <algorithm>
#include <functional>

#include <gazebo/common/common.hh>

namespace gazebo {

namespace {
constexpr double kDefaultStiffness = 330.0;
constexpr double kDefaultDamping = 20.0;
}  // namespace

void GazeboCableStretchPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;

  if (!sdf->HasElement("stretchJoint")) {
    gzerr << "[GazeboCableStretchPlugin] No <stretchJoint> elements specified on model '"
          << model_->GetName() << "', plugin has nothing to do.\n";
    return;
  }

  sdf::ElementPtr joint_elem = sdf->GetElement("stretchJoint");
  while (joint_elem) {
    const std::string joint_name = joint_elem->Get<std::string>("name");

    physics::JointPtr joint = model_->GetJoint(joint_name);
    if (!joint) {
      gzerr << "[GazeboCableStretchPlugin] Joint '" << joint_name << "' not found on model '"
            << model_->GetName() << "', skipping.\n";
      joint_elem = joint_elem->GetNextElement("stretchJoint");
      continue;
    }
    stretch_joints_.push_back(joint);

    joint_elem = joint_elem->GetNextElement("stretchJoint");
  }

  stiffness_ = sdf->HasElement("stiffness") ? sdf->Get<double>("stiffness") : kDefaultStiffness;
  damping_   = sdf->HasElement("damping")   ? sdf->Get<double>("damping")   : kDefaultDamping;

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboCableStretchPlugin::OnUpdate, this));

  gzmsg << "[GazeboCableStretchPlugin] Loaded on model '" << model_->GetName() << "' with "
        << stretch_joints_.size() << " stretch joint(s), stiffness=" << stiffness_
        << " damping=" << damping_ << ".\n";
}

void GazeboCableStretchPlugin::OnUpdate() {
  // F = -k*x - c*v along the joint's own single DOF, applied explicitly
  // every update -- see the header for why this isn't left to the URDF/SDF
  // <dynamics> spring_stiffness/spring_reference extension.
  //
  // A cable can only pull, never push: clamped the same way
  // ekf_cable_load's own tension formula already is
  // (measurement_model.cpp's `std::max(0.0, cable_stiffness * ... )`), so
  // a slack cable (x <= 0 -- the normal resting state whenever the drone
  // and load are close together, e.g. sitting on the ground) applies zero
  // force instead of acting like a compression spring shoving them apart.
  // Without this, resting/armed-but-grounded drones took a real,
  // continuous outward push (tens of Newtons at several cm of
  // "negative stretch"), which built into the drift/oscillation seen
  // before every takeoff this session.
  for (const physics::JointPtr& joint : stretch_joints_) {
    const double x = joint->Position(0);
    const double v = joint->GetVelocity(0);
    // A slack cable (x <= 0) exerts no force; otherwise the damping term would
    // pull on a still-slack cable whenever it closes fast.
    const double stretch_force = x > 0.0 ? std::max(0.0, stiffness_ * x + damping_ * v) : 0.0;
    joint->SetForce(0, -stretch_force);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboCableStretchPlugin)

}  // namespace gazebo
