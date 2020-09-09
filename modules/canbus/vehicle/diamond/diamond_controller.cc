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

#include <stdio.h>
#include <cmath>
#include <cstdio>

#include "cyber/common/log.h"

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/canbus/vehicle/diamond/diamond_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/proto/vehicle_signal.pb.h"
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
static constexpr double kEpsilon = 1e-6;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

FILE* p = nullptr;

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

  id_0x00aa5701_ = dynamic_cast<Id0x00aa5701*>(
      message_manager_->GetMutableProtocolDataById(Id0x00aa5701::ID));
  if (id_0x00aa5701_ == nullptr) {
    AERROR << "Id0x00aa5701 does not exist in the DiamondMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Id0x0c079aa7::ID, id_0x0c079aa7_, false);
  can_sender_->AddMessage(Id0x0c19f0a7::ID, id_0x0c19f0a7_, false);
  can_sender_->AddMessage(Id0x0cfff3a7::ID, id_0x0cfff3a7_, false);
  can_sender_->AddMessage(Id0x00aa5701::ID, id_0x00aa5701_, false);

  // need sleep to ensure all messages received
  AINFO << "DiamondController is initialized.";

  // Initialize frequency converter
  device_front_frequency =
      std::make_unique<Uart>(FLAGS_front_steer_device.c_str());
  device_rear_frequency =
      std::make_unique<Uart>(FLAGS_rear_steer_device.c_str());
  device_front_frequency->SetOpt(9600, 8, 'N', 1);
  device_rear_frequency->SetOpt(9600, 8, 'N', 1);

  is_initialized_ = true;
  return ErrorCode::OK;
}

DiamondController::~DiamondController() {
  // destory device resources
  device_front_frequency = nullptr;
  device_rear_frequency = nullptr;
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

  if (diamond->id_0x01().angle_sensor_id() == 1) {
    chassis_.set_front_encoder_angle(
        static_cast<float>(diamond->id_0x01().angle_sensor_data()));
    if (std::isnan(chassis_.front_encoder_angle())) {
      front_encoder_angle_realtime = front_encoder_angle_previous;
    } else {
      // if (diamond->id_0x01().angle_sensor_data() > 360.0){
      //  front_encoder_angle_realtime = 360.0;
      //} else if (diamond->id_0x01().angle_sensor_data() < 0.0){
      //  front_encoder_angle_realtime = 0.0;
      //} else{
      front_encoder_angle_realtime =
          static_cast<float>(diamond->id_0x01().angle_sensor_data());
      //}
    }
    front_wheel_angle_realtime = update_wheel_angle(
        front_wheel_angle_previous, front_encoder_angle_previous,
        front_encoder_angle_realtime, encoder_to_wheel_gear_ratio);
    chassis_.set_front_wheel_angle(front_wheel_angle_realtime);
    front_encoder_angle_previous = front_encoder_angle_realtime;
    front_wheel_angle_previous = front_wheel_angle_realtime;
    p = fopen("/home/nvidia/out.txt", "a+");
    fprintf(p, "%f\t%f\t\n", chassis_.front_wheel_angle(),
            chassis_.front_encoder_angle());
    fclose(p);
  } else {
    chassis_.set_rear_encoder_angle(
        static_cast<float>(diamond->id_0x01().angle_sensor_data()));
    if (std::isnan(chassis_.rear_encoder_angle())) {
      rear_encoder_angle_realtime = rear_encoder_angle_previous;
    } else {
      if (diamond->id_0x01().angle_sensor_data() > 360.0) {
        rear_encoder_angle_realtime = 360.0;
      } else if (diamond->id_0x01().angle_sensor_data() < 0.0) {
        rear_encoder_angle_realtime = 0.0;
      } else {
        rear_encoder_angle_realtime =
            static_cast<float>(diamond->id_0x01().angle_sensor_data());
      }
    }
    rear_wheel_angle_realtime = update_wheel_angle(
        rear_wheel_angle_previous, rear_encoder_angle_previous,
        rear_encoder_angle_realtime, encoder_to_wheel_gear_ratio);
    chassis_.set_rear_wheel_angle(rear_wheel_angle_realtime);
    rear_encoder_angle_previous = rear_encoder_angle_realtime;
    rear_wheel_angle_previous = rear_wheel_angle_realtime;
  }

  // Magnetic sensor data
  // front
  // Send messages before receive
  if (FLAGS_magnetic_enable) {
    std::string cmd = "cansend can0 003#0102030405010000";
    const int ret = std::system(cmd.c_str());
    if (ret == 0) {
      AINFO << "SUCCESS: " << cmd;
    } else {
      AERROR << "FAILED(" << ret << "): " << cmd;
    }
  }

  if (diamond->id_0x03().has_front_mgs()) {
    auto dev = getLatdev(diamond->id_0x03().front_mgs());
    if (!std::isnan(dev)) {
      chassis_.set_front_lat_dev(dev);
    }
  } else {
    chassis_.set_front_lat_dev(0);
  }
  // rear
  if (FLAGS_magnetic_enable) {
    std::string cmd4 = "cansend can0 004#0102030405010000";
    const int ret4 = std::system(cmd4.c_str());
    if (ret4 == 0) {
      AINFO << "SUCCESS: " << cmd4;
    } else {
      AERROR << "FAILED(" << ret4 << "): " << cmd4;
    }
  }
  if (diamond->id_0x04().has_rear_mgs()) {
    auto dev = getLatdev(diamond->id_0x04().rear_mgs());
    if (!std::isnan(dev)) {
      chassis_.set_rear_lat_dev(dev);
    }
  } else {
    chassis_.set_rear_lat_dev(0);
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
#if 0
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
        // p = fopen("/sys/class/gpio/gpio351/direction", "w");
        // fprintf(p, "%s", "high");
        // fclose(p);
        id_0x00aa5701_->set_relay2(0x01);
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
          // p = fopen("/sys/class/gpio/gpio271/direction", "w");
          // fprintf(p, "%s", "high");
          // fclose(p);
          id_0x00aa5701_->set_relay1(0x01);
          sleep(3);
          AERROR << "K2 down";
          // p = fopen("/sys/class/gpio/gpio351/direction", "w");
          // fprintf(p, "%s", "low");
          // fclose(p);
          id_0x00aa5701_->set_relay2(0);
        } else if (abs(chassis_detail.diamond().id_0x1818d0f3().fbatvolt() -
                       chassis_detail.diamond().id_0x0c09a7f0().fmotvolt()) >
                   25) {
          sleep(3);
          AERROR << ">25 K2 down";
          // p = fopen("/sys/class/gpio/gpio351/direction", "w");
          // fprintf(p, "%s", "low");
          // fclose(p);
          id_0x00aa5701_->set_relay2(0);
        }
      } else {
        AERROR << "1818d0f3 bybatinsrerr REEOR!!";
      }
    }
  } else {
    AERROR << chassis_detail.diamond().id_0x0c0ba7f0().dwmcuerrflg();
  }
#endif
  /*=====================k1 k2 end==========================*/
  // Driver Motor
  id_0x0c19f0a7_->set_fmot1targettq(0);
  id_0x0c19f0a7_->set_fmot1lmtvolt(800);
  id_0x0c19f0a7_->set_fmot1lmtcur(250);
  id_0x0c19f0a7_->set_bymot1workmode(0);
  id_0x0c19f0a7_->set_bylife(0);

  // Steering Motor
  SetBatCharging();

  // Steering const speed
  unsigned char spd_cmd[8] = {0x0B, 0x06, 0x20, 0x00, 0x27, 0x10, 0x98, 0x9C};
  int result_spd_positive = device_front_frequency->Write(spd_cmd, 8);
  ADEBUG << "Frequency converter speed write command send result is :"
         << result_spd_positive;

  // TODO(tianchuang): Rear wheel const speed
  unsigned char spd_cmd_rear[8] = {0x0C, 0x06, 0x20, 0x00, 0x27, 0x10, 0x99, 0x2B};
  int result_spd_positive_rear = device_rear_frequency.Write(spd_cmd_rear, 8);
  ADEBUG << "Rear frequency converter speed write command send result is :"
         << result_spd_positive_rear;

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
  // Steering stop command for 485 comm
  FrontSteerStop();
  RearSteerStop();
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(200));

  //============k1 down start===========
#if 0
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  sleep(3);
  AERROR << "1818d0f3 fbatcur="
         << chassis_detail.diamond().id_0x1818d0f3().fbatvolt();
  // p = fopen("/sys/class/gpio/gpio271/direction", "w");
  // fprintf(p, "%s", "low");
  // fclose(p);
  id_0x00aa5701_->set_relay1(0);
  AERROR << "K1 down";
  sleep(5);
#endif
  //===========k1 down end========

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
  // double real_value = params_.max_acc() * acceleration / 100;
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

  id_0x0c19f0a7_->set_fmot1targettq(std::abs(torque));
  id_0x0c19f0a7_->set_bymot1workmode(146);
}

// diamond default, -30 ~ 30, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void DiamondController::SteerFront(double front_steering_target) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  auto steering_switch = Chassis::STEERINGSTOP;

  // set steering switch by target
  if (front_steering_target > 0.1) {
    steering_switch = Chassis::STEERINGNEGATIVE;
  } else if (std::abs(front_steering_target) < 0.1) {
    steering_switch = Chassis::STEERINGSTOP;
  } else {
    steering_switch = Chassis::STEERINGPOSITIVE;
  }

  // Check wheel angle
  // TODO(all): config and enbale later
  // if (chassis_.front_wheel_angle() - 30.0 > kEpsilon ||
  //     chassis_.front_wheel_angle() + 30.0 < kEpsilon) {
  //   FrontSteerStop();
  //   return;
  // }

  switch (steering_switch) {
    case Chassis::STEERINGPOSITIVE: {
      /*
      if (std::abs(chassis_.front_wheel_angle() - front_steering_target) <
          0.5) {
        // Stop steering
        FrontSteerStop();
        break;
      }*/
      FrontSteerPositive();
      break;
    }
    case Chassis::STEERINGNEGATIVE: {
      /*
      if (abs(chassis_.front_wheel_angle() - front_steering_target) < 0.1) {
        // Stop steering
        FrontSteerStop();
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
        break;
      }*/
      FrontSteerNegative();
      break;
    }
    case Chassis::STEERINGSTOP: {
      FrontSteerStop();
      break;
    }
    default: { AINFO << "FRONT "; }
  }
}

// diamond default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void DiamondController::SteerRear(double rear_steering_target) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }

  auto steering_switch = Chassis::STEERINGSTOP;

  // set steering switch by target
  if (rear_steering_target > 0.1) {
    steering_switch = Chassis::STEERINGNEGATIVE;
  } else if (std::abs(rear_steering_target) < 0.1) {
    steering_switch = Chassis::STEERINGSTOP;
  } else {
    steering_switch = Chassis::STEERINGPOSITIVE;
  }

  switch (steering_switch) {
    case Chassis::STEERINGPOSITIVE: {
      RearSteerPositive();
      break;
    }
    case Chassis::STEERINGNEGATIVE: {
      RearSteerNegative();
      break;
    }
    default: { RearSteerStop(); }
  }
}

void DiamondController::FrontSteerStop() {
  SetBatCharging();

  unsigned char cmd[8] = {0x0B, 0x06, 0x06, 0x00, 0x00, 0x05, 0x4D, 0xA3};

  int result_dir_zero = device_front_frequency->Write(cmd, 8);
  ADEBUG << "Frequency converter direction write command send result is :"
         << result_dir_zero;
}

void DiamondController::FrontSteerPositive() {
  unsigned char cmd[8] = {0x0B, 0x06, 0x10, 0x00, 0x00, 0x01, 0x4C, 0x60};

  int result_dir_positive = device_front_frequency->Write(cmd, 8);
  ADEBUG << "Frequency converter direction write command send result is :"
         << result_dir_positive;
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
  SetBatCharging();
}

void DiamondController::FrontSteerNegative() {
  unsigned char cmd[8] = {0x0B, 0x06, 0x10, 0x00, 0x00, 0x02, 0x0C, 0x61};
  int result_dir_negative = device_front_frequency->Write(cmd, 8);
  ADEBUG << "Frequency converter direction write command send result is :"
         << result_dir_negative;
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
  SetBatCharging();
}

void DiamondController::RearSteerStop() {
  unsigned char cmd[8] = {0x0C, 0x06, 0x10, 0x00, 0x00, 0x05, 0x4C, 0x14};

  int result_dir_zero = device_rear_frequency->Write(cmd, 8);
  ADEBUG << "Frequency converter direction write command send result is :"
         << result_dir_zero;
  SetBatCharging();
}

void DiamondController::RearSteerPositive() {
  unsigned char cmd[8] = {0x0C, 0x06, 0x10, 0x00, 0x00, 0x01, 0x4D, 0xD7};

  int result_dir_positive = device_rear_frequency->Write(cmd, 8);
  ADEBUG << "Frequency converter direction write command send result is :"
         << result_dir_positive;
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
  SetBatCharging();
}

void DiamondController::RearSteerNegative() {
  unsigned char cmd[8] = {0x0C, 0x06, 0x10, 0x00, 0x00, 0x02, 0x0D, 0xD6};

  int result_dir_negative = device_rear_frequency->Write(cmd, 8);
  ADEBUG << "Frequency converter direction write command send result is :"
         << result_dir_negative;
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));
  SetBatCharging();
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

float DiamondController::update_wheel_angle(
    float wheel_angle_pre, float encoder_angle_pre, float encoder_angle_rt,
    const float encoder_to_wheel_gear_ratio) {
  float delta_encoder_angle = encoder_angle_rt - encoder_angle_pre;
  if (delta_encoder_angle < -240.0)  // 编码器发生360到0的突变，轮胎向左转
  {
    delta_encoder_angle = delta_encoder_angle + 360.0;
  } else if (delta_encoder_angle > 240.0) {
    delta_encoder_angle = delta_encoder_angle - 360.0;
  } else {
    delta_encoder_angle = delta_encoder_angle;
  }
  // delta_encoder_angle有正负，包含了左右转
  float wheel_angle_now =
      wheel_angle_pre - delta_encoder_angle / encoder_to_wheel_gear_ratio;
  wheel_angle_now = fmod(wheel_angle_now, 360.0);
  if (wheel_angle_now > 180.0) {
    wheel_angle_now = wheel_angle_now - 360.0;
  }
  return wheel_angle_now;
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
