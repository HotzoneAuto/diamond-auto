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

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/diamond/diamond_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode DiamondController::Init(
    const VehicleParameter& params,
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

  id_0x0cfff3a7_ = dynamic_cast<Id0x0cfff3a7*>(
      message_manager_->GetMutableProtocolDataById(Id0x0cfff3a7::ID));
  if (id_0x0cfff3a7_ == nullptr) {
    AERROR << "Id0x0cfff3a7 does not exist in the DiamondMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Id0x0c079aa7::ID, id_0x0c079aa7_, false);
  can_sender_->AddMessage(Id0x0c19f0a7::ID, id_0x0c19f0a7_, false);

  // need sleep to ensure all messages received
  AINFO << "DiamondController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

DiamondController::~DiamondController() {}

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

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);

  // 4 Motor torque nm
  if (chassis_detail.diamond().id_0x0c08a7f0().has_fmottq()) {
    chassis_.set_motor_torque_nm(
        static_cast<float>(chassis_detail.diamond().id_0x0c08a7f0().fmottq()));
  } else {
    chassis_.set_motor_torque_nm(0);
  }

  // 5
  // TODO(DENGKE): compute speed respect to motor torque
  // if (chassis_detail.diamond().id_0x0c08a7f0().has_fmottq()) {
  //       auto speed = 0.1 * chassis_detail.diamond().id_0x0c08a7f0().fmottq()
  //       chassis_.set_speed_mps(static_cast<float>(
  //       chassis_detail.diamond().id_0x0c08a7f0().vehicle_speed()));
  // } else {
  //   chassis_.set_speed_mps(0);
  // }

  // 7
  chassis_.set_fuel_range_m(0);

  // 8 engine rpm respect to motor speed by rpm
  if (chassis_detail.diamond().id_0x0c08a7f0().has_fmotspd()) {
    chassis_.set_engine_rpm(
        static_cast<float>(chassis_detail.diamond().id_0x0c08a7f0().fmotspd()));
  } else {
    chassis_.set_engine_rpm(0);
  }

  if (chassis_detail.diamond().id_0x1818d0f3().has_fbatvolt()) {
    chassis_.set_bat_volt(static_cast<float>(
        chassis_detail.diamond().id_0x1818d0f3().fbatvolt()));
  } else {
    chassis_.set_bat_volt(0);
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

  // Driver Motor TODO(zongbao): test on board
  id_0x0c19f0a7_->set_fmot1targettq(0);
  id_0x0c19f0a7_->set_fmot1lmtvolt(800);
  id_0x0c19f0a7_->set_fmot1lmtcur(250);
  id_0x0c19f0a7_->set_bymot1workmode(0);
  id_0x0c19f0a7_->set_bylife(0);

  // Steering Motor
  id_0x0c079aa7_->set_bydcdccmd(0xAA);
  id_0x0c079aa7_->set_bydcaccmd(0xAA);
  id_0x0c079aa7_->set_bydcacwkst(0xAA);
  id_0x0c079aa7_->set_byeapcmd(0xAA);
  id_0x0c079aa7_->set_bydcac2cmd(0xAA);
  id_0x0c079aa7_->set_bydcac2wkst(0xAA);

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
  // DC/DC
  // id_0x0c079aa7_->set_bydcdccmd(0x55);
  // DC/AC
  id_0x0c079aa7_->set_bydcaccmd(0x55);
  // DC/AC
  // id_0x0c079aa7_->set_bydcacwkst(0x55);
  // DC/AC
  // id_0x0c079aa7_->set_byeapcmd(0x55);

  // DC/DC
  id_0x0c079aa7_->set_bydcac2cmd(0x55);
  // DC/AC
  // id_0x0c079aa7_->set_bydcac2wkst(0x55);

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
  // Driver Motor TODO(zongbao): test on board
  // id_0x0c19f0a7_->set_bymot1workmode(0x92);

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

// NEUTRAL, REVERSE, DRIVE
void DiamondController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      gear_66_->set_gear_neutral();
      break;
    }
    case Chassis::GEAR_REVERSE: {
      gear_66_->set_gear_reverse();
      break;
    }
    case Chassis::GEAR_DRIVE: {
      gear_66_->set_gear_drive();
      break;
    }
    case Chassis::GEAR_PARKING: {
      gear_66_->set_gear_park();
      break;
    }
    case Chassis::GEAR_LOW: {
      gear_66_->set_gear_low();
      break;
    }
    case Chassis::GEAR_NONE: {
      gear_66_->set_gear_none();
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      gear_66_->set_gear_none();
      break;
    }
    default: {
      gear_66_->set_gear_none();
      break;
    }
  }
  */
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void DiamondController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_pedal(pedal);
  */
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void DiamondController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }

  id_0x0c19f0a7_->set_fmot1targettq(pedal);
  id_0x0c19f0a7_->set_bymot1workmode(146);
}

// confirm the car is driven by acceleration command or throttle/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void DiamondController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
}

// diamond default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void DiamondController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  // const double real_angle = 360.0 * angle / 100.0;
  // reverse sign
  // id_0x0c079aa7_->set_bydcaccmd(real_angle);
  // id_0x0c079aa7_->set_bydcac2cmd(real_angle);
  // DC/DC
  id_0x0c079aa7_->set_bydcdccmd(0x55);
  // DC/AC
  id_0x0c079aa7_->set_bydcaccmd(0x55);
  // DC/AC
  id_0x0c079aa7_->set_bydcacwkst(0x55);
  // DC/AC
  id_0x0c079aa7_->set_byeapcmd(0x55);

  // DC/DC
  id_0x0c079aa7_->set_bydcac2cmd(0x55);
  // DC/AC
  id_0x0c079aa7_->set_bydcac2wkst(0x55);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void DiamondController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  // const double real_angle = 360 * angle / 100.0;

  // id_0x0c079aa7_->set_bydcaccmd(real_angle);
  // id_0x0c079aa7_->set_bydcac2cmd(real_angle);
}

void DiamondController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void DiamondController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void DiamondController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void DiamondController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  auto signal = command.signal().turn_signal();
  if (signal == Signal::TURN_LEFT) {
    turnsignal_68_->set_turn_left();
  } else if (signal == Signal::TURN_RIGHT) {
    turnsignal_68_->set_turn_right();
  } else {
    turnsignal_68_->set_turn_none();
  }
  */
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
