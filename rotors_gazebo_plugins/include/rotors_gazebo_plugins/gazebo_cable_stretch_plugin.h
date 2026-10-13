#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_CABLE_STRETCH_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_CABLE_STRETCH_PLUGIN_H

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

/// \brief  Applies an explicit spring-damper force along a set of prismatic
///         "cable stretch" joints each physics update, via SetForce().
///
///         Exists because URDF's <dynamics> only standardizes damping/
///         friction -- spring_stiffness/spring_reference is a Gazebo-
///         specific extension, and whether the URDF->SDF conversion this
///         project's spawn path uses actually honors it was never
///         confirmed. Computing and applying F = -k*x - c*v here directly
///         removes that ambiguity: this plugin owns both the spring and
///         the damping term for the joint outright, so the joint's own
///         <dynamics> block should be left without damping/spring_* to
///         avoid double-counting.
///
///         k/c are meant to match ekf_cable_load_params.yaml's
///         cable_stiffness/cable_damping exactly, same rationale as
///         gazebo_generic_drag_plugin matching drag_coeff_x/y/z/h -- the
///         simulated physics and the estimator's own model should be the
///         same law, not two things hoped to agree.
///
///         SDF usage (repeated per joint):
///           <plugin name="cable_stretch" filename="librotors_gazebo_cable_stretch_plugin.so">
///             <stretchJoint><name>rope_1_stretch_joint</name></stretchJoint>
///             <stretchJoint><name>rope_2_stretch_joint</name></stretchJoint>
///             <stretchJoint><name>rope_3_stretch_joint</name></stretchJoint>
///             <stiffness>330</stiffness>
///             <damping>20</damping>
///           </plugin>
class GazeboCableStretchPlugin : public ModelPlugin {
 public:
  GazeboCableStretchPlugin() = default;
  ~GazeboCableStretchPlugin() override = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  void OnUpdate();

  physics::ModelPtr model_;
  std::vector<physics::JointPtr> stretch_joints_;
  double stiffness_ = 0.0;
  double damping_ = 0.0;
  event::ConnectionPtr update_connection_;
};

}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_CABLE_STRETCH_PLUGIN_H
