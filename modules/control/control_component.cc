#include "modules/control/control_component.h"

#include <string>
#include "math.h"

#include "cyber/cyber.h"

#include "modules/common/util/message_util.h"
#include "modules/control/control_wheel_angle_real.h"

namespace apollo {
namespace control {

std::string ControlComponent::Name() const { return FLAGS_control_node_name; }

ControlComponent::ControlComponent() {}

bool ControlComponent::Init() {
  ACHECK(
      cyber::common::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";
  AINFO << "deadzone:" << control_conf_.torque_deadzone();
  AINFO << "pid_conf:" << control_conf_.pid_conf().DebugString();

  // Chassis Reader
  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
        chassis_.CopyFrom(*chassis);
      });

  // front rfid Reader
  rfid_front_reader_ = node_->CreateReader<RFID>(
      FLAGS_rfid_front_topic, [this](const std::shared_ptr<RFID>& rfid_front) {
        rfid_front_.CopyFrom(*rfid_front);
      });

  // rear rfid Reader
  rfid_rear_reader_ = node_->CreateReader<RFID>(
      FLAGS_rfid_rear_topic, [this](const std::shared_ptr<RFID>& rfid_rear) {
        rfid_rear_.CopyFrom(*rfid_rear);
      });

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

  // front wheel angle Reader
  front_wheel_angle_reader_ = node_->CreateReader<WheelAngle>(
      FLAGS_front_wheel_angle_topic,
      [this](const std::shared_ptr<WheelAngle>& front_wheel_angle) {
        front_wheel_angle_.CopyFrom(*front_wheel_angle);
      });

  // rear wheel angle Reader
  rear_wheel_angle_reader_ = node_->CreateReader<WheelAngle>(
      FLAGS_rear_wheel_angle_topic,
      [this](const std::shared_ptr<WheelAngle>& rear_wheel_angle) {
        rear_wheel_angle_.CopyFrom(*rear_wheel_angle);
      });

  // TODO(tianchuang):Routing Reader

  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());
  pad_msg_.set_action(control_conf_.action());

  // create Writer
  control_cmd_writer_ =
      node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
  return true;
}

double ControlComponent::PidSpeed() {
  double pid_e = FLAGS_desired_v - static_cast<double>(chassis_.speed_mps());

  pid_int += std::isnan(pid_e) ? 0 : pid_e;

  ADEBUG << "pid_e:" << pid_e << " pid_int:" << pid_int;
  auto pid_conf = control_conf_.pid_conf();
  double torque = 0;
  torque = control_conf_.torque_deadzone() + pid_conf.kp() * pid_e +
           pid_conf.ki() * pid_int + pid_conf.kd() * (pid_e - pid_e_pre);

  ADEBUG << "PID Output Torque：" << torque
         << " vehicle speed now:" << chassis_.speed_mps();

  pid_e_pre = pid_e;

  return std::isnan(torque) ? 0 : torque;
}

// write to channel
bool ControlComponent::Proc() {
  float front_lat_dev_mgs = 0.0;
  float rear_lat_dev_mgs = 0.0;
  front_target_pre = front_wheel_angle_.value();
  rear_target_pre = rear_wheel_angle_.value();
  // front_wheel_angle_realtime = front_wheel_angle_.value();
  // rear_wheel_angle_realtime = rear_wheel_angle_.value();

  while (!apollo::cyber::IsShutdown()) {
    auto cmd = std::make_shared<ControlCommand>();
    // update Drive mode by action
    // if (pad_received_) {
    cmd->mutable_pad_msg()->CopyFrom(pad_msg_);
    // pad_received_ = false;

    // TODO: add control strategy when emergency.

    // TODO(zongbao):how to know direction(reverse or forward)
    // from station A to B (case 1: 1->2) and B to A (case 0: 2->1)
    AINFO << "rfid_front_.id=" << rfid_front_.id();
    AINFO << "rfid_rear_.id=" << rfid_rear_.id();
    switch (control_conf_.drivemotor_flag()) {
      case 1: {
        if (rfid_front_.id() == control_conf_.front_destnation()) {
          cmd->set_brake(control_conf_.soft_estop_brake());
        } else {
          if (cmd->pad_msg().action() == DrivingAction::START) {
            drivemotor_torque = PidSpeed();
	  }
	  // TODO: drivemotor_torque to config
	  if (drivemotor_torque > 46.0) {
	    drivemotor_torque = 46.0;
	  }
          cmd->set_torque(drivemotor_torque);
        }
        // TODO：检测到车速为0，驻车系统断气刹，车辆驻车停止
        break;
      }

      case 2: {
        if (rfid_rear_.id() == control_conf_.rear_destnation()) {
          cmd->set_brake(control_conf_.soft_estop_brake());
        } else {
          if (cmd->pad_msg().action() == DrivingAction::START) {
            drivemotor_torque = PidSpeed();
	  }
	  // TODO: drivemotor_torque to config
	  if (drivemotor_torque > 46.0) {
	    drivemotor_torque = 46.0;
	  }
          cmd->set_torque(-drivemotor_torque);
        }

        // TODO：检测到车速为0，驻车系统断气刹，车辆驻车停止
        break;
      }
    }

    switch (FLAGS_magnetic_enable) {
      case 1: {
        // 初始化前后磁导航检测到的偏差值，订阅磁导航通道的数据
        front_lat_dev_mgs = chassis_.front_lat_dev();
        rear_lat_dev_mgs = chassis_.rear_lat_dev();

        // 给定驱动电机反转命令（使车辆前进从A到B）
        if (control_conf_.drivemotor_flag() == 1) {
          if (!rear_wheel_wakeup) {
            cmd->set_rear_wheel_target(rear_wheel_angle_.value());    
            rear_wheel_wakeup = true;
          } else {
            cmd->set_rear_wheel_target(0);
          }
          if (front_lat_dev_mgs < -3.5)  //若前方磁导航检测出车偏左
          {
            // front_motor_steering_dir = 1;  //则前方转向电机正转（即向右）
            cmd->set_front_wheel_target(20.0);
            front_target_pre = 20.0;
          } else if (front_lat_dev_mgs > 3.5)  //若前方磁导航检测出车偏右
          {
            // front_motor_steering_dir = 2;  //则前方转向电机反转（即向左）
            cmd->set_front_wheel_target(-20.0);
            front_target_pre = -20.0;
          } else if (std::abs(front_lat_dev_mgs) < 0.1) {
            cmd->set_front_wheel_target(front_target_pre);
	      } else {
            cmd->set_front_wheel_target(0);
            front_target_pre = 0;
          }
        } else if (control_conf_.drivemotor_flag() ==
                   2)  // 若驱动电机正转（倒车，车辆从B到A）
        {
	      cmd->set_front_wheel_target(0);
          if (rear_lat_dev_mgs < -3.5)  //若后方磁导航检测出车偏左
          {
            // rear_motor_steering_dir = 1;  //则后方转向电机正转（即向右）
            cmd->set_rear_wheel_target(20.0);
            rear_target_pre = 20.0;
          } else if (rear_lat_dev_mgs > 3.5)  //若后方磁导航检测出车偏右
          {
            // rear_motor_steering_dir = 2;  //则后方转向电机反转（即向左）
            cmd->set_rear_wheel_target(-20.0);
            rear_target_pre = -20.0;
          } else if (std::abs(rear_lat_dev_mgs) < 0.1) {
            cmd->set_rear_wheel_target(rear_target_pre);
          } else {
            cmd->set_rear_wheel_target(0);
            rear_target_pre = 0;
          }
        }
        break;
      }
      case 0: {
        if (control_conf_.drivemotor_flag() == 1) {
          // rear_motor_steering_dir = 0;
          cmd->set_rear_wheel_target(0);
          cmd->set_front_wheel_target(
              control_conf_.manual_front_wheel_target());
        } else if (control_conf_.drivemotor_flag() == 2) {
          // front_motor_steering_dir = 0;
          cmd->set_front_wheel_target(0);
          cmd->set_rear_wheel_target(control_conf_.manual_rear_wheel_target());
        }
        break;
      }
      default: {}
    }

    common::util::FillHeader(node_->Name(), cmd.get());
    AINFO << "cmd: " << cmd->DebugString();
    control_cmd_writer_->Write(cmd);
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));
  }
  return true;
}

ControlComponent::~ControlComponent() {
  // back chassis handle
  async_action_.wait();
}

}  // namespace control
}  // namespace apollo
