// Standalone ROS<->Gazebo bridge for live wind control in sim: subscribes
// to /sim/wind_cmd (geometry_msgs/Vector3, world-frame) and republishes
// onto the Gazebo transport topic gazebo_generic_drag_plugin listens on,
// so wind changes at runtime without restarting sim.
//
// Adds band-limited turbulence on top of the commanded mean wind: an
// independent Ornstein-Uhlenbeck process per axis,
//   gust(t+dt) = phi*gust(t) + sigma_step*N(0,1),  phi = exp(-dt/tau)
// (same form as gazebo_imu_plugin.cpp's IMU bias random walk), with
// sigma = turbulence_intensity_fraction * |mean_wind|. A simplified Dryden
// Wind Turbulence Model stand-in (shape only, no altitude/airspeed
// spectral shaping). Rotor wake/downwash interaction between the 3 drones
// is a separate, larger effect, NOT modeled here.

#include <algorithm>
#include <cmath>
#include <random>
#include <string>

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace {
constexpr char kDefaultRosTopic[] = "/sim/wind_cmd";
constexpr char kDefaultGazeboTopic[] = "/sim/wind_velocity_gz";
}  // namespace

class WindBridge {
 public:
  WindBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& gazebo_topic)
      : random_generator_(std::random_device{}()) {
    gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gz_node_->Init();
    gz_pub_ = gz_node_->Advertise<gazebo::msgs::Vector3d>(gazebo_topic);
    ros_sub_ = nh.subscribe(kDefaultRosTopic, 1, &WindBridge::OnRosWindCmd, this);

    pnh.param("wind_default_x", mean_wind_[0], 0.0);
    pnh.param("wind_default_y", mean_wind_[1], 0.0);
    pnh.param("wind_default_z", mean_wind_[2], 0.0);
    pnh.param("turbulence_intensity_fraction", turbulence_intensity_fraction_, 0.0);
    pnh.param("turbulence_time_constant_s", turbulence_time_constant_s_, 1.0);
    double update_rate_hz = 50.0;
    pnh.param("turbulence_update_rate_hz", update_rate_hz, update_rate_hz);

    // Publish the default wind immediately so the pipeline is live from
    // the start. This specific publish can be silently dropped by a
    // Gazebo-transport connection race (found live: the drag plugin got
    // stuck at zero wind); the always-on timer below is the fix.
    PublishCurrentWind();

    // Always publish periodically, not just when turbulence is enabled --
    // self-heals the startup race above once the connection establishes.
    const double effective_rate_hz =
        (turbulence_intensity_fraction_ > 0.0) ? update_rate_hz : std::min(update_rate_hz, 2.0);
    if (effective_rate_hz > 0.0) {
      timer_ = nh.createTimer(ros::Duration(1.0 / effective_rate_hz), &WindBridge::OnTimerTick, this);
    }
  }

 private:
  void OnRosWindCmd(const geometry_msgs::Vector3::ConstPtr& msg) {
    mean_wind_[0] = msg->x;
    mean_wind_[1] = msg->y;
    mean_wind_[2] = msg->z;
    // Publish immediately for responsiveness; the periodic timer still
    // runs as the fallback against the startup race in the constructor.
    PublishCurrentWind();
  }

  void OnTimerTick(const ros::TimerEvent& event) {
    double dt = (event.current_real - event.last_real).toSec();
    if (!(dt > 0.0)) dt = 1.0 / 50.0;  // guard: first tick, or a backward/zero clock jump

    const double mean_speed = std::sqrt(mean_wind_[0] * mean_wind_[0] +
                                        mean_wind_[1] * mean_wind_[1] +
                                        mean_wind_[2] * mean_wind_[2]);
    const double sigma = turbulence_intensity_fraction_ * mean_speed;
    const double phi = std::exp(-dt / turbulence_time_constant_s_);
    const double sigma_step = sigma * std::sqrt(std::max(0.0, 1.0 - phi * phi));

    for (double& g : gust_) {
      g = phi * g + sigma_step * normal_dist_(random_generator_);
    }
    PublishCurrentWind();
  }

  void PublishCurrentWind() {
    gazebo::msgs::Vector3d gz_msg;
    gz_msg.set_x(mean_wind_[0] + gust_[0]);
    gz_msg.set_y(mean_wind_[1] + gust_[1]);
    gz_msg.set_z(mean_wind_[2] + gust_[2]);
    gz_pub_->Publish(gz_msg);
  }

  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr gz_pub_;
  ros::Subscriber ros_sub_;
  ros::Timer timer_;

  double mean_wind_[3] = {0.0, 0.0, 0.0};
  double gust_[3] = {0.0, 0.0, 0.0};
  double turbulence_intensity_fraction_ = 0.0;
  double turbulence_time_constant_s_ = 1.0;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_dist_{0.0, 1.0};
};

int main(int argc, char** argv) {
  gazebo::client::setup(argc, argv);
  ros::init(argc, argv, "wind_bridge_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string gazebo_topic;
  pnh.param<std::string>("gazebo_wind_topic", gazebo_topic, kDefaultGazeboTopic);

  WindBridge bridge(nh, pnh, gazebo_topic);
  ROS_INFO_STREAM("[wind_bridge_node] Bridging " << kDefaultRosTopic << " (ROS) -> " << gazebo_topic
                                                  << " (Gazebo), with band-limited turbulence added"
                                                  << " on top of the commanded mean wind.");

  ros::spin();

  gazebo::client::shutdown();
  return 0;
}
