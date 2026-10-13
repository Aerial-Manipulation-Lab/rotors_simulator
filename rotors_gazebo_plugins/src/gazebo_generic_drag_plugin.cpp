#include "rotors_gazebo_plugins/gazebo_generic_drag_plugin.h"

#include <functional>

#include <gazebo/common/common.hh>

namespace gazebo {

namespace {
constexpr char kDefaultWindTopic[] = "/sim/wind_velocity_gz";
}

void GazeboGenericDragPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ = model;
  world_ = model->GetWorld();

  if (!sdf->HasElement("dragLink")) {
    gzerr << "[GazeboGenericDragPlugin] No <dragLink> elements specified on model '"
          << model_->GetName() << "', plugin has nothing to do.\n";
    return;
  }

  sdf::ElementPtr link_elem = sdf->GetElement("dragLink");
  while (link_elem) {
    const std::string link_name = link_elem->Get<std::string>("name");

    physics::LinkPtr link = model_->GetLink(link_name);
    if (!link) {
      gzerr << "[GazeboGenericDragPlugin] Link '" << link_name << "' not found on model '"
            << model_->GetName() << "', skipping.\n";
      link_elem = link_elem->GetNextElement("dragLink");
      continue;
    }

    DragLink drag_link;
    drag_link.link = link;
    if (link_elem->HasElement("cx")) {
      drag_link.anisotropic = true;
      drag_link.cx = link_elem->Get<double>("cx");
      drag_link.cy = link_elem->Get<double>("cy");
      drag_link.cz = link_elem->Get<double>("cz");
      drag_link.ch = link_elem->Get<double>("ch");
    } else {
      drag_link.coefficient = link_elem->Get<double>("coefficient");
    }
    drag_links_.push_back(drag_link);

    link_elem = link_elem->GetNextElement("dragLink");
  }

  const std::string wind_topic =
      sdf->HasElement("windTopic") ? sdf->Get<std::string>("windTopic") : kDefaultWindTopic;

  gz_node_ = transport::NodePtr(new transport::Node());
  gz_node_->Init();
  wind_sub_ = gz_node_->Subscribe(wind_topic, &GazeboGenericDragPlugin::OnWindMsg, this);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboGenericDragPlugin::OnUpdate, this));

  gzmsg << "[GazeboGenericDragPlugin] Loaded on model '" << model_->GetName() << "' with "
        << drag_links_.size() << " drag link(s), listening for wind on '" << wind_topic << "'.\n";
}

void GazeboGenericDragPlugin::OnWindMsg(const boost::shared_ptr<const gazebo::msgs::Vector3d>& msg) {
  const std::lock_guard<std::mutex> lock(wind_mutex_);
  wind_velocity_ = msgs::ConvertIgn(*msg);
}

void GazeboGenericDragPlugin::OnUpdate() {
  ignition::math::Vector3d wind;
  {
    const std::lock_guard<std::mutex> lock(wind_mutex_);
    wind = wind_velocity_;
  }

  for (const DragLink& drag_link : drag_links_) {
    const ignition::math::Vector3d v_rel_world = wind - drag_link.link->WorldLinearVel();
    if (!drag_link.anisotropic) {
      drag_link.link->AddForce(drag_link.coefficient * v_rel_world);
      continue;
    }
    // Anisotropic law is in the link's own body frame (DragForceDrone's
    // x/y/z convention); rotate the relative wind into body frame, apply,
    // rotate the force back for AddForce(). DragForceDrone expects
    // v_rel = drone_velocity - wind (airspeed), the opposite sign of
    // v_rel_world above -- negate before rotating, or the linear terms
    // silently flip sign (ch is unaffected, which made this easy to miss).
    const ignition::math::Quaterniond rot = drag_link.link->WorldPose().Rot();
    const ignition::math::Vector3d v_rel_body = rot.RotateVectorReverse(-v_rel_world);
    const ignition::math::Vector3d force_body(
        -drag_link.cx * v_rel_body.X(),
        -drag_link.cy * v_rel_body.Y(),
        -drag_link.cz * v_rel_body.Z() +
            drag_link.ch * (v_rel_body.X() * v_rel_body.X() + v_rel_body.Y() * v_rel_body.Y()));
    drag_link.link->AddForce(rot.RotateVector(force_body));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGenericDragPlugin)

}  // namespace gazebo
