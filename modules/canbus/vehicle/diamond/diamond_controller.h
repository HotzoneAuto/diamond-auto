/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <stdlib.h> /* system, NULL, EXIT_FAILURE */
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/common/util/uart.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/proto/wheelangle.pb.h"

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c079aa7.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c19f0a7.h"

namespace apollo {
namespace canbus {
namespace diamond {

using apollo::drivers::WheelAngle;

class DiamondController final : public VehicleController {
 public:
  explicit DiamondController(){};

  virtual ~DiamondController();

  ::apollo::common::ErrorCode Init(
      const VehicleParameter& params,
      CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
      MessageManager<::apollo::canbus::ChassisDetail>* const message_manager)
      override;

  bool Start() override;

  /**
   * @brief stop the vehicle controller.
   */
  void Stop() override;

  /**
   * @brief calculate and return the chassis.
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues.
   */
  Chassis chassis() override;

 private:
  // main logical function for operation the car enter or exit the auto driving
  void Emergency() override;
  ::apollo::common::ErrorCode EnableAutoMode() override;
  ::apollo::common::ErrorCode DisableAutoMode() override;
  ::apollo::common::ErrorCode EnableSteeringOnlyMode() override;
  ::apollo::common::ErrorCode EnableSpeedOnlyMode() override;

  // brake with new acceleration
  // acceleration:0.00~99.99, unit:
  // acceleration_spd: 60 ~ 100, suggest: 90
  void Brake(double torque, double brake) override;

  void ForwardTorque(double torque) override;

  void ReverseTorque(double torque) override;

  // steering with old angle speed
  // angle:-99.99~0.00~99.99, unit:, left:-, right:+
  void SteerFront(double front_steering_target) override;

  // steering with old angle speed
  // angle:-99.99~0.00~99.99, unit:, left:-, right:+
  void SteerRear(double rear_steering_target) override;

  void FrontSteerStop();
  void FrontSteerPositive();
  void FrontSteerNegative();

  void RearSteerStop();
  void RearSteerPositive();
  void RearSteerNegative();

  void SetBatCharging();
  void SetMotorVoltageUp();

  // set Electrical Park Brake
  void SetEpbBreak(const ::apollo::control::ControlCommand& command) override;
  void SetBeam(const ::apollo::control::ControlCommand& command) override;
  void SetHorn(const ::apollo::control::ControlCommand& command) override;
  void SetTurningSignal(
      const ::apollo::control::ControlCommand& command) override;

  void ResetProtocol();
  bool CheckChassisError();

 private:
  void SecurityDogThreadFunc();
  virtual bool CheckResponse(const int32_t flags, bool need_wait);
  void set_chassis_error_mask(const int32_t mask);
  int32_t chassis_error_mask();
  Chassis::ErrorCode chassis_error_code();
  void set_chassis_error_code(const Chassis::ErrorCode& error_code);

 private:
  // control protocol
  Id0x0c079aa7* id_0x0c079aa7_ = nullptr;
  Id0x0c19f0a7* id_0x0c19f0a7_ = nullptr;

  Chassis chassis_;
  std::unique_ptr<std::thread> thread_;
  bool is_chassis_error_ = false;

  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;

  std::mutex chassis_mask_mutex_;
  int32_t chassis_error_mask_ = 0;

  std::thread thread_mangetic_;
  std::future<void> async_action_;

  WheelAngle front_wheel_angle_;
  WheelAngle rear_wheel_angle_;
  std::shared_ptr<cyber::Reader<apollo::drivers::WheelAngle>> front_wheel_angle_reader_;
  std::shared_ptr<cyber::Reader<apollo::drivers::WheelAngle>> rear_wheel_angle_reader_;

  // 变频器设备 485通信
  std::unique_ptr<Uart> steer_front = nullptr;
  std::unique_ptr<Uart> steer_rear = nullptr;

  float front_encoder_angle_realtime = 0;
  float rear_encoder_angle_realtime = 0;
  // TODO(all): configration
  const float encoder_to_wheel_gear_ratio = 124.5;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
