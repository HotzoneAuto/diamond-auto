﻿/******************************************************************************
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

#include <stdio.h>
#include <cstdio>
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
FILE* p = NULL;

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
  can_sender_->AddMessage(Id0x0cfff3a7::ID, id_0x0cfff3a7_, false);

  // need sleep to ensure all messages received
  AINFO << "DiamondController is initialized.";


  // Initialize frequency converter
  device_frequency_converter.SetOpt(9600, 8, 'N', 1); // TODO: confirm 4 parameters.


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

  auto diamond = chassis_detail.mutable_diamond();

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3

  // 4 Motor torque nm
  if (diamond->id_0x0c08a7f0().has_fmottq()) {
    chassis_.set_motor_torque_nm(
        static_cast<float>(diamond->id_0x0c08a7f0().fmottq()));
  } else {
    chassis_.set_motor_torque_nm(0);
  }

  // 5
  // compute speed respect to motor torque
  if (diamond->id_0x0c08a7f0().has_fmotspd()) {
    auto speed = 0.001957 * diamond->id_0x0c08a7f0().fmotspd();
    chassis_.set_speed_mps(static_cast<float>(speed));
  } else {
    chassis_.set_speed_mps(0);
  }

  // 7
  chassis_.set_fuel_range_m(0);

  // vehicle id
  chassis_.mutable_vehicle_id()->set_vin(params_.vin());

  // 8 engine rpm respect to motor speed by rpm
  if (diamond->id_0x0c08a7f0().has_fmotspd()) {
    chassis_.set_motor_rpm(
        static_cast<float>(diamond->id_0x0c08a7f0().fmotspd()));
  } else {
    chassis_.set_motor_rpm(0);
  }

  if (diamond->id_0x1818d0f3().has_fbatvolt()) {
    chassis_.set_bat_volt(static_cast<float>(
        diamond->id_0x1818d0f3().fbatvolt()));
  } else {
    chassis_.set_bat_volt(0);
  }

  if (diamond->id_0x0c09a7f0().has_fmotvolt()) {
    chassis_.set_motor_volt(static_cast<float>(
        diamond->id_0x0c09a7f0().fmotvolt()));
  } else {
    chassis_.set_motor_volt(0);
  }

  if (diamond->id_0x1818d0f3().has_fbatsoc()) {
    chassis_.set_bat_percentage(
        static_cast<float>(diamond->id_0x1818d0f3().fbatsoc()));
  } else {
    chassis_.set_bat_percentage(0);
  }

  if (diamond->id_0x01().has_angle_sensor_front()) {
    chassis_.set_front_wheel_angle(
        static_cast<float>(diamond->id_0x01().angle_sensor_front()));
  } else {
    chassis_.set_front_wheel_angle(0);
  }

  if (diamond->id_0x02().has_angle_sensor_rear()) {
    chassis_.set_rear_wheel_angle(
        static_cast<float>(diamond->id_0x02().angle_sensor_rear()));
  } else {
    chassis_.set_rear_wheel_angle(0);
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
  /*=====================k1 k2 start==========================*/
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  AINFO << "0x0c0ba7f0 ="
        << chassis_detail.diamond().id_0x0c0ba7f0().dwmcuerrflg();
  AINFO << "0x0c09a7f0 ="
        << chassis_detail.diamond().id_0x0c09a7f0().has_fmotvolt();
  AERROR << "0x1818d0f3 = "
         << chassis_detail.diamond().id_0x1818d0f3().fbatvolt();
  sleep(3);
  AINFO << "0x0c0ba7f0 ="
        << chassis_detail.diamond().id_0x0c0ba7f0().dwmcuerrflg();
  AINFO << "0x0c09a7f0 ="
        << chassis_detail.diamond().id_0x0c09a7f0().fmotrectcur();
  AERROR << "0x1818d0f3 = "
         << chassis_detail.diamond().id_0x1818d0f3().fbatvolt();
  AERROR << "0x1818d0f3fbatcur = "
         << chassis_detail.diamond().id_0x1818d0f3().fbatcur();
  if (chassis_detail.diamond().id_0x0c0ba7f0().dwmcuerrflg() == 0) {
    AINFO << "0x0c0ba7f0 ="
          << chassis_detail.diamond().id_0x0c0ba7f0().dwmcuerrflg();
    AINFO << "0x0c09a7f0 ="
          << chassis_detail.diamond().id_0x0c09a7f0().has_fmotvolt();
    id_0x0cfff3a7_->set_bybatrlyoffcmd(0);
    id_0x0cfff3a7_->set_bybatrlycmd(1);
    AERROR << "0x1818d0f3bybatnegrlysts=="
           << chassis_detail.diamond().id_0x1818d0f3().bybatnegrlysts();

    if (chassis_detail.diamond().id_0x1818d0f3().has_bybatnegrlysts() !=
            false or
        chassis_detail.diamond().id_0x1818d0f3().bybatnegrlysts() == 1) {
      if (chassis_detail.diamond().id_0x1818d0f3().bybatinsrerr() == 0) {
        AERROR << "K2 up 0x1818d0f3.bybatinsrerr=="
               << chassis_detail.diamond().id_0x1818d0f3().bybatinsrerr();
        p = fopen("/sys/class/gpio/gpio351/direction", "w");
        fprintf(p, "%s", "high");
        fclose(p);
        sleep(3);
        chassis_detail.Clear();
        message_manager_->GetSensorData(&chassis_detail);
        AERROR << "K2 up over 1818d0f3="
               << chassis_detail.diamond().id_0x1818d0f3().fbatvolt();
        AERROR << "K2 up over 0c09a7f0="
               << chassis_detail.diamond().id_0x0c09a7f0().fmotvolt();
        AERROR << "K2 up over 0c09a7f0="
               << chassis_detail.diamond().id_0x0c09a7f0().fmotrectcur();
        AERROR << " 0x0c09a7f0 fmotvolt ="
               << chassis_detail.diamond().id_0x0c09a7f0().fmotvolt();
        if (abs(chassis_detail.diamond().id_0x1818d0f3().fbatvolt() -
                chassis_detail.diamond().id_0x0c09a7f0().fmotvolt()) < 25) {
          AERROR << "K1 up";
          p = fopen("/sys/class/gpio/gpio271/direction", "w");
          fprintf(p, "%s", "high");
          fclose(p);
          sleep(3);
          AERROR << "K2 down";
          p = fopen("/sys/class/gpio/gpio351/direction", "w");
          fprintf(p, "%s", "low");
          fclose(p);
        } else if (abs(chassis_detail.diamond().id_0x1818d0f3().fbatvolt() -
                       chassis_detail.diamond().id_0x0c09a7f0().fmotvolt()) >
                   25) {
          sleep(3);
          AERROR << ">25 K2 down";
          p = fopen("/sys/class/gpio/gpio351/direction", "w");
          fprintf(p, "%s", "low");
          fclose(p);
        }
      } else {
        AERROR << "1818d0f3 bybatinsrerr REEOR!!";
      }
    }
  } else {
    AERROR << chassis_detail.diamond().id_0x0c0ba7f0().dwmcuerrflg();
  }
  /*=====================k1 k2 end==========================*/
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
  //============k1 down start===========
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  sleep(3);
  AERROR << "1818d0f3 fbatcur="
         << chassis_detail.diamond().id_0x1818d0f3().fbatvolt();
  p = fopen("/sys/class/gpio/gpio271/direction", "w");
  fprintf(p, "%s", "low");
  fclose(p);
  AERROR << "K1 down";
  sleep(5);

  if (chassis_detail.diamond().id_0x1818d0f3().fbatvolt() < 25) {
    AERROR << "K1 down over";
  } else {
    AERROR << "1818d0f3 fbatcur="
           << chassis_detail.diamond().id_0x1818d0f3().fbatvolt();
  }

  //===========k1 down end========
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

  // id_0x0c19f0a7_->set_bymot1workmode(148);
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
  // motor torque mode
  id_0x0c19f0a7_->set_bymot1workmode(146);
  // motor speed mode
  // id_0x0c19f0a7_->set_bymot1workmode(178);
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

/*
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
  // const double real_angle = 360.0 * angle / 100.0; //360 change to 45
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
*/

// diamond default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void DiamondController::Steer_Front(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  
  if (angle == 0)
  {
	// 向四合一下发报文,风机停转
    id_0x0c079aa7_->set_bydcdccmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcaccmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcacwkst(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_byeapcmd(0xAA);
    // DC/DC
    id_0x0c079aa7_->set_bydcac2cmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcac2wkst(0xAA);
  }
  else 
  {
	// 向四合一下发报文，风机启动
    id_0x0c079aa7_->set_bydcdccmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcaccmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcacwkst(0x55);
    // DC/AC
    id_0x0c079aa7_->set_byeapcmd(0xAA);
    // DC/DC
    id_0x0c079aa7_->set_bydcac2cmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcac2wkst(0xAA);
  }

  char frq_converter_dir_write_cmd[8];
  char frq_converter_spd_write_cmd[8];

  // 变频器通过485下发前转向电机正反转命令
  if (angle > 0) //need turn right
  {
	/*
	Xavier向前变频器发送：向前转向电机发送正转命令：0B 06 10 00 00 01 4C 60，同时发送额定转速命令：0B 06 20 00 27 10 98 9C （也可调速，后期标定）	
	*/    
    frq_converter_dir_write_cmd[0] = 0x0B;
    frq_converter_dir_write_cmd[1] = 0x06;
    frq_converter_dir_write_cmd[2] = 0x10;
    frq_converter_dir_write_cmd[3] = 0x00;
    frq_converter_dir_write_cmd[4] = 0x00;
    frq_converter_dir_write_cmd[5] = 0x01;
    frq_converter_dir_write_cmd[6] = 0x4C;
    frq_converter_dir_write_cmd[7] = 0x60;
    int result_dir_positive = device_.Write(frq_converter_dir_write_cmd, 8);
    ADEBUG << "Frequency converter direction write command send result is :" << result_dir_positive;

	
    frq_converter_spd_write_cmd[0] = 0x0B;
    frq_converter_spd_write_cmd[1] = 0x06;
    frq_converter_spd_write_cmd[2] = 0x20;
    frq_converter_spd_write_cmd[3] = 0x00;
    frq_converter_spd_write_cmd[4] = 0x27;
    frq_converter_spd_write_cmd[5] = 0x10;
    frq_converter_spd_write_cmd[6] = 0x98;
    frq_converter_spd_write_cmd[7] = 0x9C;
    int result_spd_positive = device_.Write(frq_converter_spd_write_cmd, 8);
    ADEBUG << "Frequency converter speed write command send result is :" << result_spd_positive;
  }
  else if (angle < 0) //need turn left
  {
	/*
	Xavier向前变频器发送：向前转向电机发送反转命令：0B 06 10 00 00 02 0C 61，同时发送额定转速命令：0B 06 20 00 27 10 98 9C （也可调速，后期标定）	
	*/
    frq_converter_dir_write_cmd[0] = 0x0B;
    frq_converter_dir_write_cmd[1] = 0x06;
    frq_converter_dir_write_cmd[2] = 0x10;
    frq_converter_dir_write_cmd[3] = 0x00;
    frq_converter_dir_write_cmd[4] = 0x00;
    frq_converter_dir_write_cmd[5] = 0x02;
    frq_converter_dir_write_cmd[6] = 0x0C;
    frq_converter_dir_write_cmd[7] = 0x61;
    int result_dir_negative = device_.Write(frq_converter_dir_write_cmd, 8);
    ADEBUG << "Frequency converter direction write command send result is :" << result_dir_negative;

	frq_converter_spd_write_cmd[0] = 0x0B;
    frq_converter_spd_write_cmd[1] = 0x06;
    frq_converter_spd_write_cmd[2] = 0x20;
    frq_converter_spd_write_cmd[3] = 0x00;
    frq_converter_spd_write_cmd[4] = 0x27;
    frq_converter_spd_write_cmd[5] = 0x10;
    frq_converter_spd_write_cmd[6] = 0x98;
    frq_converter_spd_write_cmd[7] = 0x9C;
    int result_spd_negative = device_.Write(frq_converter_spd_write_cmd, 8);
    ADEBUG << "Frequency converter speed write command send result is :" << result_spd_negative;
  }
  else // need stop turning
  {
	// Xavier向前变频器发送：向前转向电机发送停转命令：0B 06 10 00 00 05 4D A3
    frq_converter_dir_write_cmd[0] = 0x0B;
    frq_converter_dir_write_cmd[1] = 0x06;
    frq_converter_dir_write_cmd[2] = 0x10;
    frq_converter_dir_write_cmd[3] = 0x00;
    frq_converter_dir_write_cmd[4] = 0x00;
    frq_converter_dir_write_cmd[5] = 0x05;
    frq_converter_dir_write_cmd[6] = 0x4D;
    frq_converter_dir_write_cmd[7] = 0xA3;
    int result_dir_zero = device_.Write(frq_converter_dir_write_cmd, 8);
    ADEBUG << "Frequency converter direction write command send result is :" << result_dir_zero;
  }

}


// diamond default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void DiamondController::Steer_Rear(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  if (angle == 0)
  {
	// 向四合一下发报文，风机停转
    id_0x0c079aa7_->set_bydcdccmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcaccmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcacwkst(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_byeapcmd(0xAA);
    // DC/DC
    id_0x0c079aa7_->set_bydcac2cmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcac2wkst(0xAA);
  }
  else
  {
	// 向四合一下发报文，风机启动
    id_0x0c079aa7_->set_bydcdccmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcaccmd(0x55);
    // DC/AC
    id_0x0c079aa7_->set_bydcacwkst(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_byeapcmd(0xAA);
    // DC/DC
    id_0x0c079aa7_->set_bydcac2cmd(0xAA);
    // DC/AC
    id_0x0c079aa7_->set_bydcac2wkst(0xAA);
  }


  char frq_converter_dir_write_cmd[8];
  char frq_converter_spd_write_cmd[8];

  // 变频器通过485下发后转向电机正反转命令
  if (angle > 0) //need turn right
  {
	/*
	Xavier向后变频器发送：向后转向电机发送正转命令：0C 06 10 00 00 01 4D D7，同时发送额定转速命令：0C 06 20 00 27 10 99 2B （也可调速，后期标定）	
	*/    
    frq_converter_dir_write_cmd[0] = 0x0C;
    frq_converter_dir_write_cmd[1] = 0x06;
    frq_converter_dir_write_cmd[2] = 0x10;
    frq_converter_dir_write_cmd[3] = 0x00;
    frq_converter_dir_write_cmd[4] = 0x00;
    frq_converter_dir_write_cmd[5] = 0x01;
    frq_converter_dir_write_cmd[6] = 0x4D;
    frq_converter_dir_write_cmd[7] = 0xD7;
    int result_dir_positive = device_.Write(frq_converter_dir_write_cmd, 8);
    ADEBUG << "Frequency converter direction write command send result is :" << result_dir_positive;

	
    frq_converter_spd_write_cmd[0] = 0x0C;
    frq_converter_spd_write_cmd[1] = 0x06;
    frq_converter_spd_write_cmd[2] = 0x20;
    frq_converter_spd_write_cmd[3] = 0x00;
    frq_converter_spd_write_cmd[4] = 0x27;
    frq_converter_spd_write_cmd[5] = 0x10;
    frq_converter_spd_write_cmd[6] = 0x99;
    frq_converter_spd_write_cmd[7] = 0x2B;
    int result_spd_positive = device_.Write(frq_converter_spd_write_cmd, 8);
    ADEBUG << "Frequency converter speed write command send result is :" << result_spd_positive;
  }
  else if (angle < 0) //need turn left
  {
	/*
	Xavier向后变频器发送：向后转向电机发送反转命令：0C 06 10 00 00 02 0D D6，同时发送额定转速命令：0C 06 20 00 27 10 99 2B （也可调速，后期标定）	
	*/
    frq_converter_dir_write_cmd[0] = 0x0C;
    frq_converter_dir_write_cmd[1] = 0x06;
    frq_converter_dir_write_cmd[2] = 0x10;
    frq_converter_dir_write_cmd[3] = 0x00;
    frq_converter_dir_write_cmd[4] = 0x00;
    frq_converter_dir_write_cmd[5] = 0x02;
    frq_converter_dir_write_cmd[6] = 0x0D;
    frq_converter_dir_write_cmd[7] = 0xD6;
    int result_dir_negative = device_.Write(frq_converter_dir_write_cmd, 8);
    ADEBUG << "Frequency converter direction write command send result is :" << result_dir_negative;

	frq_converter_spd_write_cmd[0] = 0x0C;
    frq_converter_spd_write_cmd[1] = 0x06;
    frq_converter_spd_write_cmd[2] = 0x20;
    frq_converter_spd_write_cmd[3] = 0x00;
    frq_converter_spd_write_cmd[4] = 0x27;
    frq_converter_spd_write_cmd[5] = 0x10;
    frq_converter_spd_write_cmd[6] = 0x99;
    frq_converter_spd_write_cmd[7] = 0x2B;
    int result_spd_negative = device_.Write(frq_converter_spd_write_cmd, 8);
    ADEBUG << "Frequency converter speed write command send result is :" << result_spd_negative;
  }
  else // need stop turning
  {
	// Xavier向后变频器发送：向后转向电机发送停转命令：0C 06 10 00 00 05 4C 14
    frq_converter_dir_write_cmd[0] = 0x0C;
    frq_converter_dir_write_cmd[1] = 0x06;
    frq_converter_dir_write_cmd[2] = 0x10;
    frq_converter_dir_write_cmd[3] = 0x00;
    frq_converter_dir_write_cmd[4] = 0x00;
    frq_converter_dir_write_cmd[5] = 0x05;
    frq_converter_dir_write_cmd[6] = 0x4C;
    frq_converter_dir_write_cmd[7] = 0x14;
    int result_dir_zero = device_.Write(frq_converter_dir_write_cmd, 8);
    ADEBUG << "Frequency converter direction write command send result is :" << result_dir_zero;
  }
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
