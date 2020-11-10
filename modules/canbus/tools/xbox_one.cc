/**
 * @file xbox_one.cc
 *
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "cyber/cyber.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/common/util/message_util.h"

namespace {
// WHILE LOOPce{
using apollo::canbus::Chassis;
using apollo::common::time::Clock;
using apollo::control::ControlCommand;
using apollo::control::PadMessage;
using apollo::cyber::CreateNode;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class XboxOne {
 public:
  XboxOne() {
    ResetControlCommand();
    node_ = CreateNode("xbox_one");
  }

  void KeyboardLoopThreadFunc() {
    int32_t level = 0;
    double brake = 0;
    double front_steering = 0;
    double rear_steering = 0;
    double torque = 0;
    PadMessage pad_msg;

    // open device [in blocking mode] fd is file descriptor
    int fd = open("/dev/input/js0", O_RDONLY);
    if (fd < 0) {
      printf("Bluetooth not connected !\n");
      return;
    }

    // read number of axes
    char number_of_axes;
    ioctl(fd, JSIOCGAXES, &number_of_axes);
    // read number of buttons
    char number_of_buttons;
    ioctl(fd, JSIOCGBUTTONS, &number_of_buttons);

    // create pointers for axes and buttons
    int *axis;
    char *button;
    // allocate memory for pointers
    axis = (int *)calloc(number_of_axes, sizeof(int));
    button = (char *)calloc(number_of_buttons, sizeof(char));

    // loop that will continue to run
    while (IsRunning()) {
      // EVENT READING
      struct js_event e;
      // read data previously written to a file
      read(fd, &e, sizeof(e));
      // determine if event is of type button or axis
      // assign value to corresponding button or axis
      switch (e.type & ~JS_EVENT_INIT) {
        case JS_EVENT_BUTTON:
          button[e.number] = e.value;
          break;

        case JS_EVENT_AXIS:
          axis[e.number] = e.value;
          break;
      }
      // DISPLAY VALUES
      // printf("\r");
      // display axis numbers and values
      if (number_of_axes and level) {
        // printf("AXES: ");
        // for(int i = 0; i < number_of_axes; i++)
        // {
        // 	printf("%2d:%6d ", i, axis[i]);
        // }
        front_steering = round(axis[0] * 0.00122074);
        rear_steering = round(axis[3] * 0.00122074);
        control_command_.set_front_wheel_target(front_steering);
        control_command_.set_rear_wheel_target(rear_steering);
        // printf("\t|\t %f",front_steering);//max 32767
        // printf("\t|\t %f \t|\t",rear_steering);// -32767      1
        brake = round(axis[2] * 0.012207404);
        torque = round(axis[5] * 0.012207404);
        if (torque > 0) {
          // printf("\t|\t %f \t|\t",torque);
          control_command_.set_torque(torque);
        }
        if (brake > 0) {
          // printf("\t|\t %f \t|\t",-brake);
          control_command_.set_torque(torque);
        }
      }
      AINFO << "front_steering" << front_steering;
      AINFO << "rear_steering" << rear_steering;
      AINFO << "brake" << brake;
      AINFO << "torque" << torque;
      // display button numbers and values
      if (number_of_buttons) {
        printf("\nBUTTONS: ");
        // for(int i = 0; i < number_of_buttons; i++)
        // {
        // 	printf("%2d:%s ", i, button[i] ? "1":"0");
        // }
        if (button[6]) {
          level = 1;
        } else if (button[7]) {
          level = 0;
        }
        if (level) {
          printf("START\n");
        } else if (!level) {
          control_command_.set_front_wheel_target(0);
          control_command_.set_rear_wheel_target(0);
          control_command_.set_front_wheel_target(0);
          control_command_.set_rear_wheel_target(0);
          printf("RESET\n");
        }
        GetPadMessage(&pad_msg, level);
        control_command_.mutable_pad_msg()->CopyFrom(pad_msg);
        // sleep(1);
        // control_command_.clear_pad_msg();
      }
      // flush
      fflush(stdout);
    }
  }
  void GetPadMessage(PadMessage *pad_msg, int32_t int_action) {
    apollo::control::DrivingAction action =
        apollo::control::DrivingAction::RESET;
    switch (int_action) {
      case 0:
        action = apollo::control::DrivingAction::RESET;
        AINFO << "SET Action RESET";
        break;
      case 1:
        action = apollo::control::DrivingAction::START;
        AINFO << "SET Action START";
        break;
      default:
        AINFO << "unknown action: " << int_action << " use default RESET";
        break;
    }
    pad_msg->set_action(action);
    return;
  }
  void Send() {
    apollo::common::util::FillHeader("control", &control_command_);
    control_command_writer_->Write(control_command_);
    ADEBUG << "Control Command send OK:" << control_command_.ShortDebugString();
  }
  void ResetControlCommand() {
    control_command_.Clear();
    control_command_.set_torque(0.0);
    control_command_.set_throttle(0.0);
    control_command_.set_brake(0.0);
    control_command_.set_steering_rate(0.0);
    control_command_.set_front_wheel_target(0.0);
    control_command_.set_rear_wheel_target(0.0);
    control_command_.set_parking_brake(false);
    control_command_.set_speed(0.0);
    control_command_.set_acceleration(0.0);
    control_command_.set_engine_on_off(false);
  }

  void OnChassis(const Chassis &chassis) { Send(); }

  int32_t Start() {
    if (is_running_) {
      AERROR << "Already running.";
      return -1;
    }
    is_running_ = true;
    chassis_reader_ = node_->CreateReader<Chassis>(
        FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis> &chassis) {
          OnChassis(*chassis);
        });
    control_command_writer_ =
        node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    xbox_one_thread_.reset(
        new std::thread([this] { KeyboardLoopThreadFunc(); }));
    if (xbox_one_thread_ == nullptr) {
      AERROR << "Unable to create can client receiver thread.";
      return -1;
    }

    return 0;
  }
  void Stop() {
    if (is_running_) {
      is_running_ = false;
    }
  }
  bool IsRunning() const { return is_running_; }

 private:
  std::unique_ptr<std::thread> xbox_one_thread_;
  std::shared_ptr<Reader<Chassis>> chassis_reader_=nullptr;
  std::shared_ptr<Writer<ControlCommand>> control_command_writer_=nullptr;
  ControlCommand control_command_;
  bool is_running_ = false;
  std::shared_ptr<apollo::cyber::Node> node_;
};
}  // namespace
int main(int32_t argc, char **argv) {
  apollo::cyber::Init(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  XboxOne xboxone;

  if (xboxone.Start() != 0) {
    AERROR << "xbox_one start failed.";
    return -1;
  }

  apollo::cyber::WaitForShutdown();
  xboxone.Stop();
  AINFO << "Xbox_one exit done.";
}
