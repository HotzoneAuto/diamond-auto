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

#include "modules/canbus/vehicle/diamond/diamond_controller.h"

#include <chrono>
#include <cmath>

#include "cyber/common/log.h"
#include "cyber/cyber.h"

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/vehicle/diamond/diamond_message_manager.h"
#include "modules/canbus/vehicle/diamond/protocol/frequency_converter.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/proto/vehicle_signal.pb.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/magnetic/magnetic.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;
using apollo::drivers::canbus::SenderMessage;
using apollo::drivers::magnetic::Magnetic;

namespace {

const int32_t kMaxFailAttempt = 10;
static constexpr double kEpsilon = 1e-6;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode DiamondController::Init(
    const VehicleParameter& params, std::shared_ptr<apollo::cyber::Node> node,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "DiamondController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (node == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  id_0x0c079aa7_ = dynamic_cast<Id0x0c079aa7*>(
      message_manager_->GetMutableProtocolDataById(Id0x0c079aa7::ID));
  if (id_0x0c079aa7_ == nullptr) {
    AERROR << "Id0x0c079aa7 does not exist in the DiamondMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  id_0x0c19f0a7_ = dynamic_cast<Id0x0c19f0a7*>(
      message_manager_->GetMutableProtocolDataById(Id0x0c19f0a7::ID));
  if (id_0x0c19f0a7_ == nullptr) {
    AERROR << "Id0x0c19f0a7 does not exist in the DiamondMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Id0x0c079aa7::ID, id_0x0c079aa7_, false);
  can_sender_->AddMessage(Id0x0c19f0a7::ID, id_0x0c19f0a7_, false);

  AINFO << "DiamondController is initialized.";

  steer_front = std::make_unique<Uart>(FLAGS_front_steer_device.c_str());
  steer_rear = std::make_unique<Uart>(FLAGS_rear_steer_device.c_str());
  steer_front->SetOpt(38400, 8, 'N', 1);
  steer_rear->SetOpt(38400, 8, 'N', 1);

  // wheel angle Reader
  // remove to canbus_component
  front_wheel_angle_reader_ = node->CreateReader<WheelAngle>(
      FLAGS_front_wheel_angle_topic,
      [this](const std::shared_ptr<WheelAngle>& front_wheel_angle) {
        front_wheel_angle_.CopyFrom(*front_wheel_angle);
      });
  rear_wheel_angle_reader_ = node->CreateReader<WheelAngle>(
      FLAGS_rear_wheel_angle_topic,
      [this](const std::shared_ptr<WheelAngle>& rear_wheel_angle) {
        rear_wheel_angle_.CopyFrom(*rear_wheel_angle);
      });

  if (FLAGS_magnetic_enable) {
    apollo::drivers::magnetic::Magnetic magnetic;
    thread_mangetic_ =
        std::thread(&apollo::drivers::magnetic::Magnetic::AsyncSend, magnetic);
  }

  is_initialized_ = true;
  return ErrorCode::OK;
}

DiamondController::~DiamondController() {
  steer_front = nullptr;
  steer_rear = nullptr;

  thread_mangetic_.join();
}

bool DiamondController::Start() {
  if (!is_initialized_) {
    AERROR << "DiamondController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void DiamondController::Stop() {
  if (!is_initialized_) {
    AERROR << "DiamondController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "DiamondController stopped.";
  }
}

Chassis DiamondController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  auto diamond = chassis_detail.mutable_diamond();

  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3 Motor torque nm
  if (diamond->id_0x0c08a7f0().has_fmottq()) {
    chassis_.set_motor_torque_nm(
        static_cast<float>(diamond->id_0x0c08a7f0().fmottq()));
  } else {
    chassis_.set_motor_torque_nm(0);
  }

  // 4 compute speed respect to motor torque
  if (diamond->id_0x0c08a7f0().has_fmotspd()) {
    auto speed = 0.006079 * diamond->id_0x0c08a7f0().fmotspd();
    chassis_.set_speed_mps(static_cast<float>(speed));
  } else {
    chassis_.set_speed_mps(0);
  }

  // 5
  chassis_.set_fuel_range_m(0);

  // 6 vehicle id
  chassis_.mutable_vehicle_id()->set_vin(params_.vin());

  // 7 engine rpm respect to motor speed by rpm
  if (diamond->id_0x0c08a7f0().has_fmotspd()) {
    chassis_.set_motor_rpm(
        static_cast<float>(diamond->id_0x0c08a7f0().fmotspd()));
  } else {
    chassis_.set_motor_rpm(0);
  }

  if (diamond->id_0x1818d0f3().has_fbatvolt()) {
    chassis_.set_bat_volt(
        static_cast<float>(diamond->id_0x1818d0f3().fbatvolt()));
  } else {
    chassis_.set_bat_volt(0);
  }

  if (diamond->id_0x0c09a7f0().has_fmotvolt()) {
    chassis_.set_motor_volt(
        static_cast<float>(diamond->id_0x0c09a7f0().fmotvolt()));
  } else {
    chassis_.set_motor_volt(0);
  }

  if (diamond->id_0x1818d0f3().has_fbatsoc()) {
    chassis_.set_bat_percentage(
        static_cast<float>(diamond->id_0x1818d0f3().fbatsoc()));
  } else {
    chassis_.set_bat_percentage(0);
  }

  if (diamond->id_0x18fe0010().has_parking_mode()) {
    chassis_.set_parking_brake(diamond->id_0x18fe0010().parking_mode());
  } else {
    chassis_.set_parking_brake(false);
  }

  // Magnetic sensor data front
  // Send messages before receive
  // 1. default by system(cansend), but not best practice
  // 2. async thread by duration
  auto dev_front =
      apollo::drivers::magnetic::getLatdev(diamond->id_0x03().front_mgs());
  if (!std::isnan(dev_front)) {
    chassis_.set_front_lat_dev(dev_front);
  }

  // rear
  auto dev_rear =
      apollo::drivers::magnetic::getLatdev(diamond->id_0x04().rear_mgs());
  if (!std::isnan(dev_rear)) {
    chassis_.set_rear_lat_dev(dev_rear);
  }

  return chassis_;
}

void DiamondController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}
ErrorCode DiamondController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }

  // Driver Motor
  id_0x0c19f0a7_->set_fmot1targettq(0);
  id_0x0c19f0a7_->set_fmot1lmtvolt(800);
  id_0x0c19f0a7_->set_fmot1lmtcur(250);
  id_0x0c19f0a7_->set_bymot1workmode(0);
  id_0x0c19f0a7_->set_bylife(0);

  // Steering Motor
  SetBatCharging();

  // Steering const speed set
  int result_front = steer_front->Write(C1, 8);
  ADEBUG << "Front Steer const speed command send result:" << result_front;

  int result_rear = steer_rear->Write(C5, 8);
  ADEBUG << "Rear Steer const speed command send result:" << result_rear;

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode DiamondController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  // Steering stop command for 485
  for (int i = 0; i < 1000; i++) {
    FrontSteerStop();
    RearSteerStop();
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(15));
  }
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode DiamondController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  // Steering Motor
  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_STEER_ONLY);
  AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
  return ErrorCode::OK;
}

ErrorCode DiamondController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
  return ErrorCode::OK;
}

// brake with new torque
void DiamondController::Brake(double torque, double brake) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }

  // set Brake by tarque
  if (torque > kEpsilon) {
    id_0x0c19f0a7_->set_bymot1workmode(140);
  } else {
    id_0x0c19f0a7_->set_bymot1workmode(148);
  }

  id_0x0c19f0a7_->set_fmot1targettq(std::abs(brake));
}

void DiamondController::ForwardTorque(double torque) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  id_0x0c19f0a7_->set_fmot1targettq(torque);
  id_0x0c19f0a7_->set_bymot1workmode(138);
}

void DiamondController::ReverseTorque(double torque) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  AINFO << "entry reverse torque";

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  auto speed = 0.006079 * chassis_detail.diamond().id_0x0c08a7f0().fmotspd();
  AINFO << "speed:" << speed;


  // Fixed workmode switch bug for motor 1e-6
  if (torque < kEpsilon && speed > kEpsilon) {
    AWARN << "Skip speed error situation";
    return;
  }

  id_0x0c19f0a7_->set_fmot1targettq(std::abs(torque));
  id_0x0c19f0a7_->set_bymot1workmode(146);
}

void DiamondController::SteerFront(double front_steering_target) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }

  auto steering_switch = Chassis::STEERINGSTOP;

  // set steering switch by target
  if (front_steering_target - front_wheel_angle_.value() > 2) {
    steering_switch = Chassis::STEERINGPOSITIVE;
  } else if (std::abs(front_steering_target - front_wheel_angle_.value()) < 2) {
    steering_switch = Chassis::STEERINGSTOP;
  } else {
    steering_switch = Chassis::STEERINGNEGATIVE;
  }

  while (front_wheel_angle_.value() - 35.0 > kEpsilon) {
    steering_switch = Chassis::STEERINGNEGATIVE;
  }
  while (front_wheel_angle_.value() + 35.0 < kEpsilon) {
    steering_switch = Chassis::STEERINGPOSITIVE;
  }
  AINFO << "Steer front steering_switch = " << steering_switch;
  switch (steering_switch) {
    case Chassis::STEERINGPOSITIVE: {
      FrontSteerPositive();
      break;
    }
    case Chassis::STEERINGNEGATIVE: {
      FrontSteerNegative();
      break;
    }
    default: {
      FrontSteerStop();
      break;
    }
  }
}

void DiamondController::SteerRear(double rear_steering_target) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }

  auto steering_switch = Chassis::STEERINGSTOP;
  // set steering switch by target
  if (rear_steering_target - rear_wheel_angle_.value() > 2) {
    steering_switch = Chassis::STEERINGPOSITIVE;
  } else if (std::abs(rear_steering_target - rear_wheel_angle_.value()) < 2) {
    steering_switch = Chassis::STEERINGSTOP;
  } else {
    steering_switch = Chassis::STEERINGNEGATIVE;
  }

  while (rear_wheel_angle_.value() - 35.0 > kEpsilon) {
    steering_switch = Chassis::STEERINGNEGATIVE;
  }
  while (rear_wheel_angle_.value() + 35.0 < kEpsilon) {
    steering_switch = Chassis::STEERINGPOSITIVE;
  }
  AINFO << "Steer rear steering_switch = " << steering_switch;
  switch (steering_switch) {
    case Chassis::STEERINGPOSITIVE: {
      RearSteerPositive();
      break;
    }
    case Chassis::STEERINGNEGATIVE: {
      RearSteerNegative();
      break;
    }
    default: {
      RearSteerStop();
      break;
    }
  }
}

void DiamondController::FrontSteerStop() {
  SetBatCharging();
  int result = steer_front->Write(C2, 8);
  ADEBUG << "FrontSteerStop command send result:" << result;
}

void DiamondController::FrontSteerPositive() {
  SetBatCharging();
  int result = steer_front->Write(C3, 8);
  ADEBUG << "FrontSteerPositive command send result:" << result;
}

void DiamondController::FrontSteerNegative() {
  SetBatCharging();
  int result = steer_front->Write(C4, 8);
  ADEBUG << "FrontSteerNegative command send result:" << result;
}

void DiamondController::RearSteerStop() {
  SetBatCharging();
  int result = steer_rear->Write(C6, 8);
  ADEBUG << "RearSteerStop command send result:" << result;
}

void DiamondController::RearSteerPositive() {
  SetBatCharging();
  int result = steer_rear->Write(C7, 8);
  ADEBUG << "RearSteerPositive command send result:" << result;
}

void DiamondController::RearSteerNegative() {
  SetBatCharging();
  int result = steer_rear->Write(C8, 8);
  ADEBUG << "RearSteerNegative command send result:" << result;
}

void DiamondController::SetBatCharging() {
  id_0x0c079aa7_->set_bydcdccmd(0x55);
  id_0x0c079aa7_->set_bydcaccmd(0xAA);
  id_0x0c079aa7_->set_bydcacwkst(0xAA);
  id_0x0c079aa7_->set_byeapcmd(0xAA);
  id_0x0c079aa7_->set_bydcac2cmd(0xAA);
  id_0x0c079aa7_->set_bydcac2wkst(0xAA);
}

void DiamondController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void DiamondController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool DiamondController::CheckChassisError() { return false; }

void DiamondController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Failed to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = absl::ToUnixMicros(::apollo::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false)) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = absl::ToUnixMicros(::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR
          << "Too much time consumption in DiamondController looping process:"
          << elapsed.count();
    }
  }
}

bool DiamondController::CheckResponse(const int32_t flags, bool need_wait) {
  return true;
}

void DiamondController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t DiamondController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode DiamondController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void DiamondController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
