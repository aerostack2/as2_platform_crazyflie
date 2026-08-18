// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/*!*******************************************************************************************
 *  \file       crazyflie_platform.hpp
 *  \brief      Header for the crazyflie_platform interface.
 *  \authors    Miguel Granero Ramos
 *              Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#ifndef AS2_PLATFORM_CRAZYFLIE__CRAZYFLIE_PLATFORM_HPP_
#define AS2_PLATFORM_CRAZYFLIE__CRAZYFLIE_PLATFORM_HPP_

#include <Crazyflie.h>
#include <Eigen/Dense>

#include <memory>
#include <string>
#include <vector>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/bool.hpp"

struct logBattery
{
  float pm_vbat;
  uint8_t charge_percent;
} __attribute__((packed));

class CrazyfliePlatform : public as2::AerialPlatform
{
  std::string base_frame_;
  std::string odom_frame_;

public:
  /**
   * @brief Connect to the Crazyflie, start its log blocks and create the
   * publishers, subscriptions and timers of the platform.
   */
  void init();

  /**
   * @brief Construct the Crazyflie platform with the default namespace and the
   * radio uri taken from the uri parameter.
   */
  CrazyfliePlatform();

  /**
   * @brief Construct the Crazyflie platform in a given namespace.
   *
   * @param ns Namespace of the drone.
   */
  explicit CrazyfliePlatform(const std::string & ns);

  /**
   * @brief Construct the Crazyflie platform in a given namespace, overriding
   * the radio uri.
   *
   * @param ns Namespace of the drone.
   * @param radio_uri Crazyradio uri of the vehicle.
   */
  explicit CrazyfliePlatform(const std::string & ns, const std::string & radio_uri);

  /**
   * @brief Declare and read the platform parameters.
   *
   * @param radio_uri Unused. The uri is read from the uri parameter.
   */
  void configureParams(const std::string & radio_uri = "");

  /*  --  AS2 FUNCTIONS --  */

  void configureSensors() override;

  /**
   * @brief Arm or disarm the vehicle.
   *
   * @param state True to arm, false to disarm.
   * @return true if the vehicle accepted the request.
   */
  bool ownSetArmingState(bool state) override;
  /**
   * @brief Enter or leave offboard control.
   *
   * @param offboard True to take control, false to release it.
   * @return true if the vehicle accepted the request.
   */
  bool ownSetOffboardControl(bool offboard) override;
  /**
   * @brief Accept a control mode requested through the platform interface.
   *
   * @param msg Requested control mode.
   * @return true if the platform accepts the mode.
   */
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;
  /**
   * @brief Send the current actuator commands to the vehicle.
   *
   * @return true if the command was sent.
   */
  bool ownSendCommand() override;
  /**
   * @brief Stop the motors immediately, without landing.
   */
  void ownKillSwitch() override {cf_->emergencyStop();}
  /**
   * @brief Hold the vehicle in place with a zero setpoint.
   */
  void ownStopPlatform() override {cf_->sendStop();}

  /*  --  CRAZYFLIE FUNCTIONS --  */

  /**
   * @brief Log the variables the connected Crazyflie exposes, for debugging.
   */
  void listVariables();

  /**
   * @brief Poll the radio link, dispatching the pending log packets.
   */
  void pingCB();

  /**
   * @brief Publish the IMU measurement of a gyro and accelerometer log packet.
   *
   * @param time_in_ms Timestamp of the packet, in ms since the vehicle booted.
   * @param values {gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z}.
   */
  void onLogIMU(uint32_t time_in_ms, std::vector<double> * values, void * /*userData*/);

  /**
   * @brief Store the orientation of an odometry log packet.
   *
   * @param time_in_ms Timestamp of the packet, in ms since the vehicle booted.
   * @param values Roll, pitch and yaw of the vehicle.
   */
  void onLogOdomOri(uint32_t time_in_ms, std::vector<double> * values, void * /*userData*/);

  /**
   * @brief Store the position of an odometry log packet and publish the odometry.
   *
   * @param time_in_ms Timestamp of the packet, in ms since the vehicle booted.
   * @param values Position and linear velocity of the vehicle.
   */
  void onLogOdomPos(uint32_t time_in_ms, std::vector<double> * values, void * /*userData*/);

  /**
   * @brief Publish the battery level read from the vehicle.
   */
  void onLogBattery();

  /**
   * @brief Publish the measurement of a range deck log packet.
   *
   * @param time_in_ms Timestamp of the packet, in ms since the vehicle booted.
   * @param values Ranges of the multiranger deck.
   */
  void onLogRange(uint32_t time_in_ms, std::vector<double> * values, void * /*userData*/);

  /**
   * @brief Publish the odometry built from the last stored position and
   * orientation log packets.
   */
  void updateOdom();

  /**
   * @brief Forward an external pose estimate to the Crazyflie estimator.
   *
   * @param msg Pose of the vehicle, from an external localization system.
   */
  void externalOdomCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /*  --  AUX FUNCTIONS --  */

  Eigen::Vector3d quaternion2Euler(geometry_msgs::msg::Quaternion quat);

private:
  std::shared_ptr<Crazyflie> cf_;
  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::TimerBase::SharedPtr bat_timer_;
  bool is_connected_;
  bool is_armed_;
  std::string uri_;
  uint8_t controller_type_;
  uint8_t estimator_type_;
  bool enable_multiranger_;

  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;

  /*  --  SENSORS --  */

  float initial_yaw_ = 0.0;
  bool has_initial_yaw_ = false;

  // Odometry
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odom_estimate_ptr_;
  double odom_buff_[10];
  std::function<void(uint32_t, std::vector<double> *, void *)> cb_odom_ori_;
  std::shared_ptr<LogBlockGeneric> odom_logBlock_ori_;
  bool ori_rec_;

  std::function<void(uint32_t, std::vector<double> *, void *)> cb_odom_pos_;
  std::shared_ptr<LogBlockGeneric> odom_logBlock_pos_;
  bool pos_rec_;

  // IMU
  std::unique_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  double imu_buff_[6];
  std::function<void(uint32_t, std::vector<double> *, void *)> cb_imu_;
  std::shared_ptr<LogBlockGeneric> imu_logBlock_;

  // Battery
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::BatteryState>> battery_sensor_ptr_;
  unsigned char battery_buff_;

  /* std::unique_ptr<LogBlock<struct logBattery>> bat_logBlock_;
  std::function<void(uint32_t, struct logBattery *)> cb_bat_; */

  // Optitrack
  bool external_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr external_odom_sub_;
  std::string external_odom_topic_;

  // Multi-ranger deck
  std::unique_ptr<as2::sensors::Sensor<sensor_msgs::msg::LaserScan>> multi_ranger_sensor_ptr_;
  double range_buff_[6];
  std::function<void(uint32_t, std::vector<double> *, void *)> cb_range_;
  std::shared_ptr<LogBlockGeneric> range_logBlock_;
};

#endif  // AS2_PLATFORM_CRAZYFLIE__CRAZYFLIE_PLATFORM_HPP_
