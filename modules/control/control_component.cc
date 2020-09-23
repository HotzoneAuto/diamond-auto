#include "modules/control/control_component.h"

#include <string>
#include "math.h"

#include "cyber/cyber.h"

#include "modules/common/util/message_util.h"

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
        front_wheel_angle_value = front_wheel_angle->value();
        AINFO << "front_wheel_angle.value() = " << front_wheel_angle->value();
        is_front_received = true;
      });

  // rear wheel angle Reader
  rear_wheel_angle_reader_ = node_->CreateReader<WheelAngle>(
      FLAGS_rear_wheel_angle_topic,
      [this](const std::shared_ptr<WheelAngle>& rear_wheel_angle) {
        rear_wheel_angle_value = rear_wheel_angle->value();
        AINFO << "rear_wheel_angle.value() = " << rear_wheel_angle->value();
        is_rear_received = true;
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
  double pid_e =
      control_conf_.desired_v() - static_cast<double>(chassis_.speed_mps());

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

double ControlComponent::GetSteerTarget(float lat_dev_mgs,
                                        double& target_last) {
  double wheel_target;
  if (lat_dev_mgs < -3.5) {
    wheel_target = 20.0;
  } else if (lat_dev_mgs > 3.5) {
    wheel_target = -20.0;
  } else if (std::abs(lat_dev_mgs) < 0.1) {
    wheel_target = target_last;
  } else if (std::abs(lat_dev_mgs) <= 3.5) {
    wheel_target = 0;
  }
  target_last = wheel_target;
  return wheel_target;
}

// write to channel
bool ControlComponent::Proc() {
  while (!apollo::cyber::IsShutdown()) {
    auto cmd = std::make_shared<ControlCommand>();
    // update Drive mode by action
    cmd->mutable_pad_msg()->CopyFrom(pad_msg_);

    // TODO: add control strategy when emergency.

    // TODO(zongbao):how to know direction(reverse or forward)
    AINFO << "rfid_front_.id=" << rfid_front_.id();
    AINFO << "rfid_rear_.id=" << rfid_rear_.id();

    // TODO: Routing Module automatically decides drivemotor_flag
    if (control_conf_.drivemotor_flag() == 1) {
      // longitudinal control
      if (rfid_front_.id() == control_conf_.front_destnation()) {
        is_destination = true;
      } else {
        if (cmd->pad_msg().action() == DrivingAction::START) {
          drivemotor_torque = PidSpeed();
        }
      }

      // lateral control
      if (FLAGS_magnetic_enable) {
        AINFO << "front_wheel_angle_value = " << front_wheel_angle_value;

        float front_lat_dev_mgs = chassis_.front_lat_dev();
        float rear_lat_dev_mgs = chassis_.rear_lat_dev();
        AINFO << "front_lat_dev_mgs = " << front_lat_dev_mgs;
        AINFO << "rear_lat_dev_mgs = " << rear_lat_dev_mgs;

        // rear_wheel_target = 0;

        // TODO: test wakeup with original mgs data
        if (!front_wheel_wakeup && is_front_received &&
            cmd->pad_msg().action() == START) {
          front_target_last = front_wheel_angle_value;
          front_wheel_target = front_wheel_angle_value;
          limit_front_wheel = false;
          front_wheel_wakeup = true;
          AINFO << "front_target_last = " << front_target_last;
          AINFO << "front wheel wake up.";
        } else {
          front_wheel_target =
              GetSteerTarget(front_lat_dev_mgs, front_target_last);
          limit_front_wheel = true;
        }
      } else {
        // rear_wheel_target = 0;
        front_wheel_target = control_conf_.manual_front_wheel_target();
      }

      // set control cmd
      // check estop, ture: brake=10,torque=1, write
      if (is_destination) {
        cmd->set_brake(control_conf_.soft_estop_brake());
        cmd->set_torque(1);
        cmd->set_rear_wheel_target(rear_wheel_angle_value);
        cmd->set_front_wheel_target(front_wheel_angle_value);
      } else {
        drivemotor_torque = (drivemotor_torque < control_conf_.max_torque())
                                ? drivemotor_torque
                                : control_conf_.max_torque();
        drivemotor_torque =
            (drivemotor_torque > 0.001) ? drivemotor_torque : 0.001;
        if (limit_front_wheel) {
          front_wheel_target =
              (front_wheel_target < 30.0) ? front_wheel_target : 30.0;
          front_wheel_target =
              (front_wheel_target > -30.0) ? front_wheel_target : -30.0;
        }
        rear_wheel_target = -front_wheel_target;

        AINFO << "front_wheel_target = " << front_wheel_target;
        AINFO << "rear_wheel_target = " << rear_wheel_target;

        if (cmd->pad_msg().action() != DrivingAction::START) {
          AINFO << "not START, cmd set to 0";
          cmd->set_torque(0);
          cmd->set_rear_wheel_target(0);
          cmd->set_front_wheel_target(0);
        } else {
          cmd->set_torque(drivemotor_torque);
          cmd->set_rear_wheel_target(rear_wheel_target);
          cmd->set_front_wheel_target(front_wheel_target);
        }
      }
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
