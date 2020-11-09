/**
 * @file xbox_one.cc
 *
 */
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/input.h>
#include <linux/joystick.h>

#include <termios.h>
#include "cyber/cyber.h"

#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/common/util/message_util.h"

namespace{
	// WHILE LOOPce{
	using apollo::canbus::Chassis;
	using apollo::common::VehicleSignal;
	using apollo::common::time::Clock;
	using apollo::control::ControlCommand;
	using apollo::control::PadMessage;
	using apollo::cyber::CreateNode;
	using apollo::cyber::Reader;
	using apollo::cyber::Writer;
	
	class Xbox_one{
		public:
		Xbox_one(){
			ResetControlCommand();
			node_=CreateNode("xbox_one");
		}
		static void PrintKeycode(){
			// system("clear");
			printf("=====================     Handle Mapping   ===================\n");

		}
		void KeyboardLoopThreadFunc() {
			bool b=false;
			// bool d=false;
			// char c = 0;
			// int32_t level = 0;
			// // double brake = 0;
			// // double throttle = 0;
			// double front_steering = 0;
			// double rear_steering = 0;
			// double torque = 0;
			// struct termios cooked_;
			// struct termios raw_;
			// int32_t kfd_ = 0;
			// bool parking_brake = false;
			// Chassis::GearPosition gear = Chassis::GEAR_INVALID;
			PadMessage pad_msg;
			// AINFO << c ;
			

			// INIT
			// open device [in blocking mode]
			// fd is file descriptor
			int fd = open("/dev/input/js0", O_RDONLY);
			if (fd < 0)
   			{
				printf("Bluetooth not connected !\n");
				return;
   			}
			// IOCTLs
			// read number of axes
			char number_of_axes;
			ioctl(fd, JSIOCGAXES, &number_of_axes);
			// read number of buttons
			char number_of_buttons;
			ioctl(fd, JSIOCGBUTTONS, &number_of_buttons);

			// OTHER VARIABLES [NOT PART OF API DOC]
			// create pointers for axes and buttons
			int *axis;
			char *button;
			// allocate memory for pointers
			axis = (int *)calloc(number_of_axes, sizeof(int));
			button = (char *)calloc(number_of_buttons, sizeof(char));
			
			// double a;
			// WHILE LOOP
			// loop that will continue to run
			while(IsRunning())
			{

				// EVENT READING
				// creating a js_event struct called e
				// (e is of type js_event)
				// js_event is defined in joystick.h
				struct js_event e;
				// read data previously written to a file
				// read sizeof(e) bytes from file descriptor fd into buffer pointed to by &e
				read(fd, &e, sizeof(e));

				// JS_EVENT.TYPE
				// possible values for type are defined in joystick.h
				//		button pressed
				//		joystick moves
				//		init state of device

				// JS_EVENT.NUMBER
				// values of number correspond to axis or button that generated event
				// values vary from one joystick to another

				// JS_EVENT.VALUE
				// for axis, value is position of joystick along axis
				// axis values range from -32767 to 32767
				// for buttons, value is state of button
				// button values range from 0 to 1

				// JS_EVENT.TIME
				// values of time correspond to the time an event was generated

				// DETERMINE AND ASSIGN [NOT PART OF API DOC]
				// determine if event is of type button or axis
				// assign value to corresponding button or axis
				switch(e.type & ~JS_EVENT_INIT)
				{
					case JS_EVENT_BUTTON:
						button[e.number] = e.value;
						break;

					case JS_EVENT_AXIS:
						axis[e.number] = e.value;
						break;
				}

				// DISPLAY VALUES
				// move to beginning of current line
				printf("\r");
				// display axis numbers and values
				if(number_of_axes)
				{
					printf("AXES: ");
					// for(int i = 0; i < number_of_axes; i++)
					// {
					// 	printf("%2d:%6d ", i, axis[i]);
					// }
					// a=round(axis[2]*0.012207404);
					// printf("\t|\t %f",round(axis[0]*0.00122074));//max 32767 
					// printf("\t|\t %f \t|\t",round(axis[3]*0.00122074));// -32767      1       32767
					// printf("\t|\t %f \t|\t",a);//  1        200      400
					// -0.00003051850947599719
					// printf("\t|\t %f \t|\t",round(axis[5]*0.012207404));
					
					// if(a>0){
					// 	printf("\t|\t %f \t|\t",a);	
					// }

				}
				// display button numbers and values
				if(number_of_buttons)
				{
					printf("\nBUTTONS: ");
					// for(int i = 0; i < number_of_buttons; i++)
					// {
					// 	printf("%2d:%s ", i, button[i] ? "1":"0");
					// }
				if(button[6]){
					b=true;
				}else if(button[7]){
					b=false;
				}
				if(b){
					GetPadMessage(&pad_msg, button[6]);
					control_command_.mutable_pad_msg()->CopyFrom(pad_msg);
					// sleep(1);
					control_command_.clear_pad_msg();
					printf("&&&&&&&&");	
				}else if(!b){
					printf("^^^^^^^^^");
				}
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
			// control_command_.set_driving_mode(Chassis::COMPLETE_MANUAL);
			control_command_.set_gear_location(Chassis::GEAR_INVALID);
			control_command_.mutable_signal()->set_turn_signal(
				VehicleSignal::TURN_NONE);
		}

		void OnChassis(const Chassis &chassis) { Send(); }

		int32_t Start(){
		if (is_running_) {
			AERROR << "Already running.";
			return -1;
		}	
		is_running_ = true;
		// chassis_reader_ = node_->CreateReader<Chassis>(
        // FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis> &chassis) {
        //   OnChassis(*chassis);
        // });
    	// control_command_writer_ =
        // node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
    	xbox_one_thread_.reset(
        new std::thread([this] { KeyboardLoopThreadFunc(); }));
		if (xbox_one_thread_ == nullptr) {
		AERROR << "Unable to create can client receiver thread.";
		return -1;
		}

		return 0;
		}
		void Stop(){
			if (is_running_) {
				is_running_ = false;
			}
		}
		bool IsRunning() const { return is_running_; }
		private:
			std::unique_ptr<std::thread> xbox_one_thread_;
			std::shared_ptr<Reader<Chassis>> chassis_reader_;
			std::shared_ptr<Writer<ControlCommand>> control_command_writer_;
			ControlCommand control_command_;
			bool is_running_ = false;
			std::shared_ptr<apollo::cyber::Node> node_;
	};
}//namespace
int main()
{
	
	

	Xbox_one xbox_one;

	if (xbox_one.Start() != 0) {
    AERROR << "xbox_one start failed.";
    return -1;
	}	
	Xbox_one::PrintKeycode();

	apollo::cyber::WaitForShutdown();
	xbox_one.Stop();
	AINFO << "Xbox_one exit done.";
	
}
