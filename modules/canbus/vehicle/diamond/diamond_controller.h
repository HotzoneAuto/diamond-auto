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

#include <stdio.h>
#include <stdlib.h> /* system, NULL, EXIT_FAILURE */
#include <memory>
#include <string>
#include <string_view>
#include <thread>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/common/util/uart.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "modules/canbus/vehicle/diamond/protocol/id_0x00aa5701.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c079aa7.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c19f0a7.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0cfff3a7.h"

namespace apollo {
namespace canbus {
namespace diamond {

float getLatdev(int dec) {
  int bin = 0, temp = dec, j = 1;
  while (temp) {
    bin = bin + j * (temp % 2);
    temp = temp / 2;
    j = j * 10;
  }
  std::string s = std::to_string(bin);
  while (s.size() < 16) {
    s = '0' + s;
  }
  int sum_activated = 0;
  int sum_id = 0;
  for (int i = 0; i < 16; i++) {
    if (s[i] == '1') {
      sum_id += 16 - i;
      sum_activated += 1;
    }
  }
  auto lat_dev_mgs =
      static_cast<float>(sum_id) / static_cast<float>(sum_activated) - 8.5;
  return lat_dev_mgs;
}

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
  void Brake(double acceleration) override;

  void ForwardTorque(double torque) override;

  void ReverseTorque(double torque) override;

  // steering with old angle speed
  // angle:-99.99~0.00~99.99, unit:, left:-, right:+
  void SteerFront(double front_steering_target);

  // steering with old angle speed
  // angle:-99.99~0.00~99.99, unit:, left:-, right:+
  void SteerRear(double rear_steering_target) override;

  void FrontSteerStop();
  void FrontSteerPositive();
  void FrontSteerNegative();

  void RearSteerStop();
  void RearSteerPositive();
  void RearSteerNegative();

  // steering with new angle speed
  // angle:-99.99~0.00~99.99, unit:, left:+, right:-
  // angle_spd:0.00~99.99, unit:deg/s
  void Steer(double angle, double angle_spd) override;

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
  float update_wheel_angle(float wheel_angle_pre, float encoder_angle_pre,
                           float encoder_angle_rt,
                           const float encoder_to_wheel_gear_ratio);

 private:
  // control protocol
  Id0x0c079aa7* id_0x0c079aa7_ = nullptr;
  Id0x0c19f0a7* id_0x0c19f0a7_ = nullptr;
  Id0x0cfff3a7* id_0x0cfff3a7_ = nullptr;
  Id0x00aa5701* id_0x00aa5701_ = nullptr;

  Chassis chassis_;
  std::unique_ptr<std::thread> thread_;
  bool is_chassis_error_ = false;

  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;

  std::mutex chassis_mask_mutex_;
  int32_t chassis_error_mask_ = 0;

  // 变频器 485通信 设备
  Uart device_front_frequency_converter =
      Uart("ttyUSB2");  // TODO: define device name.
  Uart device_rear_frequency_converter =
      Uart("ttyUSB1");  // TODO: define device name.

  float front_encoder_angle_previous = 0;

  float front_encoder_angle_realtime = 0;
  float rear_encoder_angle_previous = 0;

  float rear_encoder_angle_realtime = 0;
  const float encoder_to_wheel_gear_ratio = 124.5;
  float front_wheel_angle_previous = 0;
  float front_wheel_angle_realtime = 0;
  float rear_wheel_angle_previous = 0;
  float rear_wheel_angle_realtime = 0;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
