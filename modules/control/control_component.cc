#include "modules/control/control_component.h"

#include <string>
#include "math.h"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/control_pid.h"
#include "modules/control/control_wheel_angle_real.h"

namespace apollo {
namespace control {

using apollo::cyber::Rate;
using apollo::cyber::Time;

bool ControlComponent::Init() {
  ACHECK(
      cyber::common::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";

  // Chassis Reader
  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
        chassis_.CopyFrom(*chassis);
      });

  // rfid Reader
  rfid_reader_ = node_->CreateReader<RFID>(
      FLAGS_rfid_topic,
      [this](const std::shared_ptr<RFID>& rfid) { rfid_.CopyFrom(*rfid); });

  // Padmsg Reader
  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      FLAGS_pad_topic, [this](const std::shared_ptr<PadMessage>& pad_msg) {
        pad_msg_.CopyFrom(*pad_msg);
        if (pad_msg != nullptr) {
          ADEBUG << "pad_msg: " << pad_msg_.ShortDebugString();
          if (pad_msg_.action() == DrivingAction::RESET) {
            AINFO << "Control received RESET action!";
          }
          pad_received_ = true;
        }
      });

  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());
  pad_msg_.set_action(control_conf_.action());

  // create Writer
  control_cmd_writer_ =
      node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);

  // compute control message in aysnc thread
  async_action_ = cyber::Async(&ControlComponent::GenerateCommand, this);

  return true;
}

double ControlComponent::PidSpeed() {
  double pid_e = FLAGS_desired_v - static_cast<double>(chassis_.speed_mps());

  pid_int += std::isnan(pid_e) ? 0 : pid_e;

  AINFO << "pid_e:" << pid_e << " pid_int:" << pid_int;
  auto pid_conf = control_conf_.pid_conf();
  double torque = 0;
  torque = control_conf_.torque_deadzone() + pid_conf.kp() * pid_e +
           pid_conf.ki() * pid_int + pid_conf.kd() * (pid_e - pid_e_pre);

  AINFO << "PID Output Torque：" << torque
        << " vehicle speed now:" << chassis_.speed_mps();

  pid_e_pre = pid_e;

  return std::isnan(torque) ? 0 : torque;
}

// write to channel
void ControlComponent::GenerateCommand() {
  // frequency
  Rate rate(100.0);

  float front_lat_dev_mgs = 0.0;
  float rear_lat_dev_mgs = 0.0;

  // 获取当前车辆速度
  veh_spd = chassis_.speed_mps();

  // 初始化驱动电机死区
  speed_motor_deadzone = control_conf_.torque_deadzone();
  front_wheel_angle_realtime = chassis_.front_wheel_angle();
  rear_wheel_angle_realtime = chassis_.rear_wheel_angle();

  while (!apollo::cyber::IsShutdown()) {
    auto cmd = std::make_shared<ControlCommand>();
    if (pad_received_) {
      cmd->mutable_pad_msg()->CopyFrom(pad_msg_);
      pad_received_ = false;
    }
    // TODO: Configuration
    // 手动给定，0代表停止，1代表从A到B，2代表从B到A
    drivemotor_flag = 1;
    switch (drivemotor_flag) {
      // 从A到B
      case 1: {
        if (rfid_.id() == 2) {
          // TODO: 制动转矩，需改成标定值
          drivemotor_torque = 10;
          cmd->set_brake(drivemotor_torque);
          cmd->set_torque(0);
        } else {
          drivemotor_torque = PidSpeed();
          cmd->set_torque(drivemotor_torque);
        }

        // TODO：检测到车速为0，驻车系统断气刹，车辆驻车停止
        break;
      }

      // 从B到A
      case 2: {
        if (rfid_.id() == 1) {
          // TODO: 制动转矩，需改成标定值
          drivemotor_torque = 10;
          cmd->set_brake(drivemotor_torque);
          cmd->set_torque(0);
        } else {
          drivemotor_torque = PidSpeed();
          cmd->set_torque(drivemotor_torque);
        }

        // TODO：检测到车速为0，驻车系统断气刹，车辆驻车停止
        break;
      }

      // 停止状态
      case 0: {
        cmd->set_torque(0);
        // TODO：检测到车速为0，驻车系统断气刹，车辆驻车停止
        break;
      }
    }

    front_wheel_angle_realtime = chassis_.front_wheel_angle();
    rear_wheel_angle_realtime = chassis_.rear_wheel_angle();

    // 检测到轮胎转角超过30°，转向电机停转
    if (std::abs(front_wheel_angle_realtime) > 30) {
      // front_motor_steering_dir = 0;
    }

    if (std::abs(rear_wheel_angle_realtime) > 30) {
      // rear_motor_steering_dir = 0;
    }

    switch (FLAGS_magnetic_enable) {
      case 1: {
        // 初始化前后磁导航检测到的偏差值，订阅磁导航通道的数据
        // TODO: 检查，共用了一个数据
        front_lat_dev_mgs = chassis_.front_lat_dev();
        rear_lat_dev_mgs = chassis_.rear_lat_dev();

        // 给定驱动电机反转命令（使车辆前进从A到B）
        if (drivemotor_flag == 1) {
          // rear_motor_steering_dir = 0;   //后方转向电机不转
          cmd->set_rear_wheel_target(0);
          if (front_lat_dev_mgs < -3.5)  //若前方磁导航检测出车偏左
          {
            // front_motor_steering_dir = 1;  //则前方转向电机正转（即向右）
            cmd->set_front_wheel_target(10.0);
          } else if (front_lat_dev_mgs > 3.5)  //若前方磁导航检测出车偏右
          {
            // front_motor_steering_dir = 2;  //则前方转向电机反转（即向左）
            cmd->set_front_wheel_target(-10.0);
          } else {
            if (front_wheel_angle_realtime >= 0.5)  // 当前前轮转角为正，向右偏
            {
              // front_motor_steering_dir = 2;  // 前方转向电机反转（向左）
              cmd->set_front_wheel_target(0);
            } else if ((front_wheel_angle_realtime > -0.5) &&
                       (front_wheel_angle_realtime < 0.5)) {
              // front_motor_steering_dir = 0;  // 前方转向电机停转
              cmd->set_front_wheel_target(0);
            } else  // 当前前轮转角为负，向左偏
            {
              // front_motor_steering_dir = 1;  // 前方转向电机正转（向右）
              cmd->set_front_wheel_target(0);
            }
          }
        } else if (drivemotor_flag == 2)  // 若驱动电机正转（倒车，车辆从B到A）
        {
          // front_motor_steering_dir = 0;  //前方转向电机不转
          cmd->set_front_wheel_target(0);
          if (rear_lat_dev_mgs < -3.5)  //若后方磁导航检测出车偏左
          {
            // rear_motor_steering_dir = 1;  //则后方转向电机正转（即向右）
            cmd->set_rear_wheel_target(10.0);
          } else if (rear_lat_dev_mgs > 3.5)  //若后方磁导航检测出车偏右
          {
            // rear_motor_steering_dir = 2;  //则后方转向电机反转（即向左）
            cmd->set_rear_wheel_target(-10.0);
          } else {
            if (rear_wheel_angle_realtime >= 0.5)  // 当前后轮转角为正，向右偏
            {
              // rear_motor_steering_dir = 2;  // 后方转向电机反转（向左）
              cmd->set_rear_wheel_target(0);
            } else if ((rear_wheel_angle_realtime > -0.5) &&
                       (rear_wheel_angle_realtime < 0.5)) {
              // rear_motor_steering_dir = 0;  // 后方转向电机停转
              cmd->set_rear_wheel_target(0);
            } else  // 当前后轮转角为负，向左偏
            {
              // rear_motor_steering_dir = 1;  // 后方转向电机正转（向右）
              cmd->set_rear_wheel_target(0);
            }
          }
        } else {  // 若出现异常
          // auto motor_torque = pid_speed(veh_spd, 0, speed_motor_deadzone);
          // front_motor_steering_dir = 0;  // 停止
          // rear_motor_steering_dir = 0;   // 停止
          cmd->set_rear_wheel_target(0);
        }
        break;
      }
      case 0: {
        if (drivemotor_flag == 1) {
          // rear_motor_steering_dir = 0;
          cmd->set_rear_wheel_target(0);
          cmd->set_front_wheel_target(
              control_conf_.manual_front_wheel_target());
        } else if (drivemotor_flag == 2) {
          // front_motor_steering_dir = 0;
          cmd->set_front_wheel_target(0);
          cmd->set_rear_wheel_target(control_conf_.manual_rear_wheel_target());
        } else {
          // front_motor_steering_dir = 0;
          // rear_motor_steering_dir = 0;
          cmd->set_front_wheel_target(0);
          cmd->set_rear_wheel_target(0);
        }
        break;
      }
      default: {}
    }

    control_cmd_writer_->Write(cmd);

    rate.Sleep();
  }
}

ControlComponent::~ControlComponent() {
  // back chassis handle
  async_action_.wait();
}

}  // namespace control
}  // namespace apollo
