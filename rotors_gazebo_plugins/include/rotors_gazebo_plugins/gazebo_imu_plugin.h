/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_GAZEBO_PLUGINS_IMU_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_IMU_PLUGIN_H

#include <random>

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Imu.pb.h"

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {

// Default values for use with ADIS16448 IMU
static constexpr double kDefaultAdisGyroscopeNoiseDensity =
    2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk =
    2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime =
    1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma =
    0.5 / 180.0 * M_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity =
    2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerRandomWalk =
    2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime =
    300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma =
    20.0e-3 * 9.8;
// Earth's gravity in Zurich (lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
static constexpr double kDefaultGravityMagnitude = 9.8068;

// A description of the parameters:
// https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
// TODO(burrimi): Should I have a minimalistic description of the params here?
struct ImuParameters {
  /// Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
  double gyroscope_noise_density;
  /// Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
  double gyroscope_random_walk;
  /// Gyroscope bias correlation time constant [s]
  double gyroscope_bias_correlation_time;
  /// Gyroscope turn on bias standard deviation [rad/s]
  double gyroscope_turn_on_bias_sigma;
  /// Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
  double accelerometer_noise_density;
  /// Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
  double accelerometer_random_walk;
  /// Accelerometer bias correlation time constant [s]
  double accelerometer_bias_correlation_time;
  /// Accelerometer turn on bias standard deviation [m/s^2]
  double accelerometer_turn_on_bias_sigma;
  /// Norm of the gravitational acceleration [m/s^2]
  double gravity_magnitude;

  ImuParameters()
      : gyroscope_noise_density(kDefaultAdisGyroscopeNoiseDensity),
        gyroscope_random_walk(kDefaultAdisGyroscopeRandomWalk),
        gyroscope_bias_correlation_time(
            kDefaultAdisGyroscopeBiasCorrelationTime),
        gyroscope_turn_on_bias_sigma(kDefaultAdisGyroscopeTurnOnBiasSigma),
        accelerometer_noise_density(kDefaultAdisAccelerometerNoiseDensity),
        accelerometer_random_walk(kDefaultAdisAccelerometerRandomWalk),
        accelerometer_bias_correlation_time(
            kDefaultAdisAccelerometerBiasCorrelationTime),
        accelerometer_turn_on_bias_sigma(
            kDefaultAdisAccelerometerTurnOnBiasSigma),
        gravity_magnitude(kDefaultGravityMagnitude) {}
};

class GazeboImuPlugin : public ModelPlugin {
 public:

  GazeboImuPlugin();
  ~GazeboImuPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief  This method adds noise to acceleration and angular rates for
  ///         accelerometer and gyroscope measurement simulation.
  void AddNoise(
      Eigen::Vector3d* linear_acceleration,
      Eigen::Vector3d* angular_velocity,
      const double dt);

  /// \brief  	This gets called by the world update start event.
  /// \details	Calculates IMU parameters and then publishes one IMU message.
  void OnUpdate(const common::UpdateInfo&);

 private:

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();



  std::string namespace_;
  std::string imu_topic_;

  /// \brief    Handle for the Gazebo node.
  transport::NodePtr node_handle_;

  transport::PublisherPtr imu_pub_;

  std::string frame_id_;
  std::string link_name_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;

  /// \brief    Pointer to the world.
  physics::WorldPtr world_;

  /// \brief    Pointer to the model.
  physics::ModelPtr model_;

  /// \brief    Pointer to the link.
  physics::LinkPtr link_;

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;

  /// \brief    IMU message.
  /// \details  This is modified everytime OnUpdate() is called,
  //            and then published onto a topic
  gz_sensor_msgs::Imu imu_message_;

  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;

  Eigen::Vector3d gyroscope_bias_;
  Eigen::Vector3d accelerometer_bias_;

  Eigen::Vector3d gyroscope_turn_on_bias_;
  Eigen::Vector3d accelerometer_turn_on_bias_;

  ImuParameters imu_parameters_;

  /// \brief    Opt-in per-IMU seed (SDF randomSeed, int, -1): >= 0 seeds the noise generator from (seed,
  ///           robotNamespace, imuTopic), so every IMU gets its own realisation and a seed reproduces it; -1 keeps the
  ///           default-constructed generator (identical realisation on every IMU and every launch).
  int random_seed_ = -1;

  /// \brief    Frame vibration (opt-in): per axis a sum of independent coloured noises, each white noise through a
  ///           2nd-order filter section (biquad, bilinear transform at the IMU rate). SDF accelerometerVibration /
  ///           gyroscopeVibration: "<x> | <y> | <z>", per axis components separated by ",":
  ///             lp2 D fc / lp4 D fc   white noise through a 2nd / 4th-order Butterworth low-pass at fc [Hz]
  ///             white D               white noise
  ///             res A f0 zeta         white noise through a band-pass with gain 1 at f0 [Hz], damping zeta
  ///           D, A: one-sided PSD level (D^2 = PSD in the pass band) in [unit/sqrt(Hz)]. Empty: no vibration.
  struct Biquad {
    double b0 = 0, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    double Step(double x) {
      const double y = b0 * x + z1;
      z1 = b1 * x - a1 * y + z2;
      z2 = b2 * x - a2 * y;
      return y;
    }
  };
  struct VibrationComponent {
    std::string type;        // lp2, lp4, white, res
    double level = 0, f0 = 0, zeta = 0;
    std::vector<Biquad> sections;
  };
  std::vector<VibrationComponent> accelerometer_vibration_[3];
  std::vector<VibrationComponent> gyroscope_vibration_[3];
  double vibration_dt_ = -1.0;
  // own generator, so switching the vibration on or off leaves the stock noise sequence of a seed unchanged
  std::mt19937 vibration_generator_;
  std::normal_distribution<double> vibration_normal_{0.0, 1.0};
  static void ParseVibration(const sdf::ElementPtr& sdf, const std::string& name,
                             std::vector<VibrationComponent> (&out)[3]);
  static void DesignVibration(double dt, std::vector<VibrationComponent>* components);
  double SampleVibration(std::vector<VibrationComponent>* components, double dt);
};

}  // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_IMU_PLUGIN_H
