#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_GENERIC_DRAG_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_GENERIC_DRAG_PLUGIN_H

#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo {

/// \brief  Applies a linear drag force to a configurable set of links on
///         one model, driven by a wind velocity that's live-updatable at
///         runtime over a Gazebo-transport topic (no respawn needed).
///
///         Two per-link force laws, chosen per <dragLink> by which SDF
///         sub-elements are present:
///
///         Isotropic (linear only), the default:
///           F = coefficient * (wind_velocity - link_velocity), world frame.
///
///         Anisotropic (matches ekf_cable_load's DragForceDrone exactly),
///         computed in the LINK's own body frame then rotated to world:
///           F_body = (-cx*vx, -cy*vy, -cz*vz + ch*(vx^2+vy^2))
///           where v = wind_velocity - link_velocity, rotated into body frame.
///
///         Isotropic is used for rope/load links (no fitted anisotropic
///         model exists for them). Anisotropic is for the 3 drone
///         base_links, with cx/cy/cz/ch set to the SAME drag_coeff_x/y/z/h
///         ekf_cable_load_params.yaml uses, so sim physics and the
///         estimator's drag model agree.
///
///         SDF usage (repeated per link):
///           <plugin name="wind_drag" filename="librotors_gazebo_generic_drag_plugin.so">
///             <windTopic>/sim/wind_velocity_gz</windTopic>  <!-- optional, this is the default -->
///             <dragLink><name>rope_1_link_0</name><coefficient>0.05</coefficient></dragLink>
///             <dragLink><name>falcon1/base_link</name>
///               <cx>0.29</cx><cy>0.31</cy><cz>0.40</cz><ch>0.02</ch>
///             </dragLink>
///             ...
///           </plugin>
///
///         Drive it from ROS via the wind_bridge_node in this package,
///         which republishes geometry_msgs/Vector3 on /sim/wind_cmd onto
///         this same Gazebo topic.
class GazeboGenericDragPlugin : public ModelPlugin {
 public:
  GazeboGenericDragPlugin() = default;
  ~GazeboGenericDragPlugin() override = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  struct DragLink {
    physics::LinkPtr link;
    bool anisotropic = false;
    double coefficient = 0.0;               // isotropic mode
    double cx = 0.0, cy = 0.0, cz = 0.0, ch = 0.0;  // anisotropic mode
  };

  void OnUpdate();
  void OnWindMsg(const boost::shared_ptr<const gazebo::msgs::Vector3d>& msg);

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  std::vector<DragLink> drag_links_;
  event::ConnectionPtr update_connection_;

  transport::NodePtr gz_node_;
  transport::SubscriberPtr wind_sub_;

  std::mutex wind_mutex_;
  ignition::math::Vector3d wind_velocity_{0.0, 0.0, 0.0};
};

}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_GENERIC_DRAG_PLUGIN_H
