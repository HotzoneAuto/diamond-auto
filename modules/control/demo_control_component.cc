#include "modules/control/demo_control_component.h"
#include "modules/control/diamondauto_control_pid.h"
#include "modules/control/diamondauto_control_wheel_angle_real.h"

#include <string>
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "math.h"

namespace apollo
{
namespace control
{

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::canbus::diamond;

bool ControlComponent::Init()
{
	// Reader
	chassis_reader_ = node_->CreateReader<Chassis>(
	                      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis)
	{
		chassis_.CopyFrom(*chassis);
	});

	// create Writer
	control_cmd_writer_ = node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);

	// compute control message in aysnc
	async_action_ = cyber::Async(&ControlComponent::GenerateCommand, this);

	return true;
}

// write to channel
void ControlComponent::GenerateCommand()
{
	auto cmd = std::make_shared<ControlCommand>();

	// frequency, TODO: reset
	Rate rate(20.0);
	
	// 初始化前后磁导航检测到的偏差值
	float front_lat_dev_mgs = 0;  // TODO: lateral_dev_mgs need to changed according to MGS module.
	float back_lat_dev_mgs = 0;

	// 获取当前车辆速度
	veh_spd = chassis_.speed_mps()

	// 获取前后编码器瞬时角度值
	front_encoder_angle_realtime = chassis_.front_encoder_angle();
	rear_encoder_angle_realtime = chassis_.rear_encoder_angle();

	/*
	// 获取前后编码器瞬时角度值
	Id0x01 angle_sensor;
	int ID_sensor = angle_sensor.angle_sensor_ID(0, 8);
	double front_encoder_angle_realtime = 0;
	double rear_encoder_angle_realtime = 0;

	if (ID_sensor == 1)
	{
		front_encoder_angle_realtime = angle_sensor.angle_sensor_data(0, 8); // 前编码器当前的瞬时角度值，单位是deg
	}
	else if (ID_sensor == 2)
	{
		rear_encoder_angle_realtime = angle_sensor.angle_sensor_data(0, 8); // 后编码器当前的瞬时角度值，单位是deg
	}
	*/



	// 初始化驱动电机死区
	if (veh_spd <= 0.1){ // TODO: 需更换成订阅canbus的车速数据
		speed_motor_deadzone = speed_motor_deadzone_calibration; // 标定值
	}
	else{
		speed_motor_deadzone = r_wheel * m_veh * g * f_c / (i1 * i0 * yita_t); // 使用滚动阻力系数，此时死区指的是理论电机需求转矩
	}	
	
	// cmd -> set_front_steering_target(0);
	// cmd -> set_back_steering_target(0); //初始时前后转向轮转角为0


	//TODO: 突然断电停车后，需记录最后时刻的状态数据，包括前后轮转角；重新启动后再读取数据文件

	
	//上过高压自检完成之后，进入自动驾驶模式后，车辆处于ready状态时
	while (front_wheel_angle_realtime > 0.5)// 待标定
	{
		front_motor_steering_dir = 2; 
		//则前方转向电机反转（即向左）
		/*
		Xavier向前变频器发送：向前转向电机发送反转命令：0B 06 10 00 00 02 0C 61，同时发送额定转速命令：0B 06 20 00 27 10 98 9C （也可调速，后期标定）
		同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA 55 AA AA AA AA，逆变器1工作，前转向电机风扇1工作
		*/
		//TODO: 消息发送，刷新

		front_wheel_angle_previous = front_wheel_angle_realtime;
		front_wheel_angle_realtime = update_wheel_angle(front_wheel_angle_previous, front_encoder_angle_previous, front_encoder_angle_realtime, encoder2wheel_gear_ratio);
		rate.Sleep(); 
	}

	while (rear_wheel_angle_realtime > 0.5)// 待标定
	{
		rear_motor_steering_dir = 2; //则后方转向电机反转（即向左）
		/*
		Xavier向后变频器发送：向后转向电机发送反转命令：0C 06 10 00 00 02 0D D6，同时发送额定转速命令：0C 06 20 00 27 10 99 2B （也可调速，后期标定）
		同时Xavier向四合一 id_0x0C079AA7 发送 AA AA 55 AA AA AA AA AA，逆变器2工作，后转向电机风扇2工作
		*/
		//TODO: 消息发送，刷新

		rear_wheel_angle_previous = rear_wheel_angle_realtime;
		rear_wheel_angle_realtime = update_wheel_angle(rear_wheel_angle_previous, rear_encoder_angle_previous, rear_encoder_angle_realtime, encoder2wheel_gear_ratio);
		rate.Sleep(); 
	}
	
	while (front_wheel_angle_realtime < -0.5)// 待标定
	{
		front_motor_steering_dir = 1; //则前方转向电机正转（即向右）
		/*
		Xavier向前变频器发送：向前转向电机发送正转命令：0B 06 10 00 00 01 4C 60，同时发送额定转速命令：0B 06 20 00 27 10 98 9C （也可调速，后期标定）
		同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA 55 AA AA AA AA，逆变器1工作，前转向电机风扇1工作
		*/
		//TODO: 消息发送，刷新

		front_wheel_angle_previous = front_wheel_angle_realtime;
		front_wheel_angle_realtime = update_wheel_angle(front_wheel_angle_previous, front_encoder_angle_previous, front_encoder_angle_realtime, encoder2wheel_gear_ratio);
		rate.Sleep();
	}

	while (rear_wheel_angle_realtime < -0.5)// 待标定
	{
		rear_motor_steering_dir = 1; //则后方转向电机正转（即向右）
		/*
		Xavier向后变频器发送：向后转向电机发送正转命令：0C 06 10 00 00 01 4D D7，同时发送额定转速命令：0C 06 20 00 27 10 99 2B （也可调速，后期标定）
		同时Xavier向四合一 id_0x0C079AA7 发送 AA AA 55 AA AA AA AA AA，逆变器2工作，后转向电机风扇2工作
		*/
		//TODO: 消息发送，刷新

		rear_wheel_angle_previous = rear_wheel_angle_realtime;
		rear_wheel_angle_realtime = update_wheel_angle(rear_wheel_angle_previous, rear_encoder_angle_previous, rear_encoder_angle_realtime, encoder2wheel_gear_ratio);
		rate.Sleep(); 
	}

	// TODO: 补充下发命令代码
	/*
		Xavier向前变频器发送：向前转向电机发送停转命令：0B 06 10 00 00 05 4D A3
		同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA AA AA AA AA AA，逆变器1停止工作，前转向电机风扇1停止工作
	*/
	//TODO: 消息发送，刷新

	/*
		Xavier向后变频器发送：向后转向电机发送停转命令：0C 06 10 00 00 05 4C 14
		同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA AA AA AA AA AA，逆变器2停止工作，后转向电机风扇2停止工作
	*/
	//TODO: 消息发送，刷新
	
	
	front_motor_steering_dir = 0; //则前方转向电机停转
	rear_motor_steering_dir = 0; //则后方转向电机停转


	while (true)
	{
		// 先只看A到B
		if(find_rfid_B == 1){//检测到B点rfid
			drivemotor_torque = pid_speed(veh_spd, 0,speed_motor_deadzone); //PID控制目标是驱动电机停转
			cmd -> set_throttle(drivemotor_torque);
		}
			// 以0为车速目标，向canbus发送经过PID后的转矩

		else if (find_rfid_B == 0){ //未检测到B点rfid
			drivemotor_torque = pid_speed(veh_spd,1,speed_motor_deadzone); //驱动电机驱动汽车以1m/s运动，这是PID目标，尽可能接近1
			cmd -> set_throttle(drivemotor_torque);
		}
			// 以1为车速目标，向canbus发送经过PID后的转矩
			
		/*
		// 遥控，人工给输入
		if (veh_dir == 0) //若行驶方向从A到B（前进，A是第2辆车处，B是机械臂处）
			drivemotor_flag = 1; 
		else if (veh_dir == 1) //若行驶方向从B到A（后退）
			drivemotor_flag = 2;
		*/

		front_wheel_angle_realtime = update_wheel_angle(front_wheel_angle_previous, front_encoder_angle_previous, front_encoder_angle_realtime, encoder2wheel_gear_ratio);
		rear_wheel_angle_realtime = update_wheel_angle(rear_wheel_angle_previous, rear_encoder_angle_previous, rear_encoder_angle_realtime, encoder2wheel_gear_ratio);

		/*
		TODO: 增加 获取当前时刻的编码器角度值
		*/



		if (/* motor_speed == 1 && */ drivemotor_flag == 1)  // 若驱动电机反转（车辆前进A到B）
		{
			rear_motor_steering_dir = 0; //后方转向电机不转
			if (front_lat_dev_mgs < -4.5)  //若前方磁导航检测出车偏左
			{
				front_motor_steering_dir = 1; //则前方转向电机正转（即向右）
				/*
				Xavier向前变频器发送：向前转向电机发送正转命令：0B 06 10 00 00 01 4C 60，同时发送额定转速命令：0B 06 20 00 27 10 98 9C （也可调速，后期标定）
				同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA 55 AA AA AA AA，逆变器1工作，前转向电机风扇1工作
				*/
				cmd -> set_front_steering_target(10); //TODO 宗宝一起确定: 10%，待标定（目标角度暂时不用）
			}
			else if (front_lat_dev_mgs > 4.5)  //若前方磁导航检测出车偏右
			{
				front_motor_steering_dir = 2; //则前方转向电机反转（即向左）
				/*
				Xavier向前变频器发送：向前转向电机发送反转命令：0B 06 10 00 00 02 0C 61，同时发送额定转速命令：0B 06 20 00 27 10 98 9C （也可调速，后期标定）
				同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA 55 AA AA AA AA，逆变器1工作，前转向电机风扇1工作
				*/
				cmd -> set_front_steering_target(-10); //TODO 宗宝一起确定: 10%，待标定（目标角度暂时不用）
			}
			else{
				if (front_wheel_angle_realtime >= 0.5) // 当前前轮转角为正，向右偏
				{
					front_motor_steering_dir = 2; // 前方转向电机反转（向左）
					/*
					Xavier向前变频器发送：向前转向电机发送反转命令：0B 06 10 00 00 02 0C 61，同时发送额定转速命令：0B 06 20 00 27 10 98 9C （也可调速，后期标定）
					同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA 55 AA AA AA AA，逆变器1工作，前转向电机风扇1工作
					*/
					cmd -> set_front_steering_target(-10); //TODO 宗宝一起确定: 目标转角为0，摆正，待标定（目标角度暂时不用）	
				}
				else if ((front_wheel_angle_realtime > -0.5) && (front_wheel_angle_realtime < 0.5)){
					front_motor_steering_dir = 0; // 前方转向电机停转
					/*
					Xavier向前变频器发送：向前转向电机发送停转命令：0B 06 10 00 00 05 4D A3
					同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA AA AA AA AA AA，逆变器1停止工作，前转向电机风扇1停止工作
					*/
					cmd -> set_front_steering_target(0); //TODO 宗宝一起确定: 目标转角为0，摆正，待标定（目标角度暂时不用）	
				}
				else // 当前前轮转角为负，向左偏
				{
					front_motor_steering_dir = 1; // 前方转向电机正转（向右）
					/*
					Xavier向前变频器发送：向前转向电机发送正转命令：0B 06 10 00 00 01 4C 60，同时发送额定转速命令：0B 06 20 00 27 10 98 9C （也可调速，后期标定）
					同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA 55 AA AA AA AA，逆变器1工作，前转向电机风扇1工作
					*/
					cmd -> set_front_steering_target(10); //TODO 宗宝一起确定: 目标转角为0，摆正，待标定（目标角度暂时不用）
				}
			}
		}
		else if (/* motor_speed == 1*/ drivemotor_flag == 2)  // 若驱动电机正转（倒车，车辆从B到A）
		{
			front_motor_steering_dir = 0; //前方转向电机不转
			if (back_lat_dev_mgs < -4.5)  //若后方磁导航检测出车偏左
			{
				rear_motor_steering_dir = 1; //则后方转向电机正转（即向右）
				/*
				Xavier向后变频器发送：向后转向电机发送正转命令：0C 06 10 00 00 01 4D D7，同时发送额定转速命令：0C 06 20 00 27 10 99 2B （也可调速，后期标定）
				同时Xavier向四合一 id_0x0C079AA7 发送 AA AA 55 AA AA AA AA AA，逆变器2工作，后转向电机风扇2工作
				*/
				cmd -> set_back_steering_target(10); //TODO 宗宝一起确定: 10%，待标定（目标角度暂时不用）
			}
			else if (front_lat_dev_mgs > 4.5)  //若后方磁导航检测出车偏右
			{
				rear_motor_steering_dir = 2; //则后方转向电机反转（即向左）
				/*
				Xavier向后变频器发送：向后转向电机发送反转命令：0C 06 10 00 00 02 0D D6，同时发送额定转速命令：0C 06 20 00 27 10 99 2B （也可调速，后期标定）
				同时Xavier向四合一 id_0x0C079AA7 发送 AA AA 55 AA AA AA AA AA，逆变器2工作，后转向电机风扇2工作
				*/
				cmd -> set_back_steering_target(-10); //TODO 宗宝一起确定: 10%，待标定（目标角度暂时不用）
			}
			else{
				if (rear_wheel_angle_realtime >= 0.5) // 当前后轮转角为正，向右偏
				{
					rear_motor_steering_dir = 2; // 后方转向电机反转（向左）
					/*
					Xavier向后变频器发送：向后转向电机发送反转命令：0C 06 10 00 00 02 0D D6，同时发送额定转速命令：0C 06 20 00 27 10 99 2B （也可调速，后期标定）
					同时Xavier向四合一 id_0x0C079AA7 发送 AA AA 55 AA AA AA AA AA，逆变器2工作，后转向电机风扇2工作
					*/
					cmd -> set_back_steering_target(-10); //TODO 宗宝一起确定: 目标转角为0，摆正，待标定（目标角度暂时不用）	
				}
				else if ((rear_wheel_angle_realtime > -0.5) && (rear_wheel_angle_realtime < 0.5)){
					rear_motor_steering_dir = 0; // 后方转向电机停转
					/*
					Xavier向后变频器发送：向后转向电机发送停转命令：0C 06 10 00 00 05 4C 14
					同时Xavier向四合一id_0x0C079AA7 发送 AA AA AA AA AA AA AA AA，逆变器2停止工作，后转向电机风扇2停止工作
					*/
					cmd -> set_back_steering_target(0); //TODO 宗宝一起确定: 目标转角为0，摆正，待标定（目标角度暂时不用）
				}
				else // 当前后轮转角为负，向左偏
				{
					rear_motor_steering_dir = 1; // 后方转向电机正转（向右）
					/*
					Xavier向后变频器发送：向后转向电机发送正转命令：0C 06 10 00 00 01 4D D7，同时发送额定转速命令：0C 06 20 00 27 10 99 2B （也可调速，后期标定）
					同时Xavier向四合一 id_0x0C079AA7 发送 AA AA 55 AA AA AA AA AA，逆变器2工作，后转向电机风扇2工作
					*/
					cmd -> set_back_steering_target(10); //TODO 宗宝一起确定: 目标转角为0，摆正，待标定（目标角度暂时不用）
				}
			}
		}
		else{ // 若出现异常
			motor_torque = pid_speed(veh_spd, 0, speed_motor_deadzone);
			front_motor_steering_dir = 0; // 停止
			back_motor_steering_dir = 0; // 停止
			cmd -> set_front_steering_target(0); 
			cmd -> set_back_steering_target(0); 
		}

		// 更新前轮转角和编码器度数
		front_wheel_angle_realtime = update_wheel_angle(front_wheel_angle_previous, front_encoder_angle_previous, front_encoder_angle_realtime, encoder2wheel_gear_ratio);
		rear_wheel_angle_realtime = update_wheel_angle(rear_wheel_angle_previous, rear_encoder_angle_previous, rear_encoder_angle_realtime, encoder2wheel_gear_ratio);

		front_wheel_angle_previous = front_wheel_angle_realtime;
		rear_wheel_angle_previous = rear_wheel_angle_realtime;

		front_encoder_angle_previous = front_encoder_angle_realtime;
		rear_encoder_angle_previous = rear_encoder_angle_realtime;

		control_cmd_writer_->Write(cmd);

		rate.Sleep();
	}
}

ControlComponent::~ControlComponent()
{
	// back chassis handle
	async_action_.wait();
}

}  // namespace control
}  // namespace apollo
