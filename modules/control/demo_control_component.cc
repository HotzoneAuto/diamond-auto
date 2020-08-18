#include "modules/control/demo_control_component.h"
#include "modules/control/diamondauto_control_pid.h"

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
	
	int find_rfid_des = 0; //记录是否检测到终点的rfid
	int dir = 0; //记录行驶方向, 0代表从A到B, 1代表从B到A
	
	// TODO：发送扭矩，PID天闯
	int motor_flag = 0; //控制驱动电机正反转
	// float motor_speed = 0; //驱动电机转速
	float motor_torque = 0; //驱动电机转矩
	
	int front_steering_dir = 0; //控制前方转向电机正反转
	int back_steering_dir = 0; //控制后方转向电机正反转
	
	// float front_steering_speed = 0; //rpm，前方转向电机转速
	// float back_steering_speed = 0; //rpm，后方转向电机转速
	
	float front_wheel_angle = 0; //前轮转角
	float back_wheel_angle = 0; //后轮转角
	
	cmd -> set_front_steering_target(0);
	cmd -> set_back_steering_target(0); //初始时前后转向电机转角为0
	
	//上过高压自检完成之后，进入自动驾驶模式后，车辆处于ready状态时
	while (front_wheel_angle > 0.5 || back_wheel_angle > 0.5)// 待标定
	{
		front_steering_dir = 0; 
		back_steering_dir = 0; //则前后方转向电机反转（即向左）
		/*
		此处应当控制前方转向电机的转速（标定值，初始为额定转速1435rpm）
		Xavier向四合一发送55 AA 55 55 AA AA AA AA 逆变器1工作，转向电机风扇1工作
		*/
		//TODO: 消息发送，刷新
	}
	
	while (front_wheel_angle < -0.5 || back_wheel_angle < -0.5)// 待标定
	{
		front_steering_dir = 1; 
		back_steering_dir = 1; //则前后方转向电机正转（即向右）
		/*
		此处应当控制前方转向电机的转速（标定值，初始为额定转速1435rpm）
		Xavier向四合一发送55 AA 55 55 AA AA AA AA 逆变器1工作，转向电机风扇1工作
		*/
		//TODO: 消息发送，刷新
	}
	
	while (true)
	{
		if (find_rfid_des == 1) //检测到终点rfid
			motor_torque = pid_speed(0); //PID控制目标是驱动电机停转
			// 以0为车速目标，向canbus发送经过PID后的转矩
		else if (find_rfid_des == 0) //未检测到终点rfid
			motor_torque = pid_speed(1); //驱动电机驱动汽车以1m/s运动，这是PID目标，尽可能接近1
			// 以1为车速目标，向canbus发送经过PID后的转矩
			
		// 遥控，人工给输入
		if (dir == 0) //若行驶方向从A到B（前进）
			motor_flag = 0;
		else if (dir == 1) //若行驶方向从B到A（后退）
			motor_flag = 1;

		if (/* motor_speed == 1 && */ motor_flag == 0)  // 若驱动电机正转（前进）
		{
			back_steering_dir = 2; //后方转向电机不转
			if (front_lat_dev_mgs < -4.5)  //若前方磁导航检测出车偏左
			{
				front_steering_dir = 0; //则前方转向电机正转（即向右）
				/*
				此处应当控制前方转向电机的转速（标定值，初始为额定转速1435rpm）
				判断角位移传感器反馈的角度不大于30°。当大于30°时，转向电机转速为0
				Xavier向四合一发送55 AA 55 AA AA AA AA AA 逆变器1工作，转向电机风扇1工作
				*/
				cmd -> set_front_steering_target(10); //转向量为10，待标定（目标角度暂时不用）
			}
			else if (front_lat_dev_mgs > 4.5)  //若前方磁导航检测出车偏右
			{
				front_steering_dir = 1; //则前方转向电机反转（即向左）
				/*
				此处应当控制前方转向电机的转速（标定值，初始为额定转速1435rpm）
				判断角位移传感器反馈的角度不大于30°。当大于30°时，转向电机转速为0
				Xavier向四合一发送55 AA 55 AA AA AA AA AA 逆变器1工作，转向电机风扇1工作
				*/
				cmd -> set_front_steering_target(10); //转向量为10，待标定（目标角度暂时不用）
			}
			else
				front_steering_dir = 2; //前方转向电机不转
		}
		else if (/* motor_speed == 1*/ motor_flag == 1)  // 若驱动电机反转（倒车）
		{
			front_steering_dir = 2; //前方转向电机不转
			if (back_lat_dev_mgs < -4.5)  //若后方磁导航检测出车偏左
			{
				back_steering_dir = 0; //则后方转向电机正转（即向右）
				/*
				此处应当控制后方转向电机的转速（标定值，初始为额定转速1435rpm）
				判断角位移传感器反馈的角度不大于30°。当大于30°时，转向电机转速为0
				Xavier向四合一发送55 AA AA 55 AA AA AA AA 逆变器1工作，转向电机风扇1工作
				*/
				cmd -> set_back_steering_target(10); //转向量为10
			}
			else if (front_lat_dev_mgs > 4.5)  //若后方磁导航检测出车偏右
			{
				back_steering_dir = 1; //则后方转向电机反转（即向左）
				/*
				此处应当控制后方转向电机的转速（标定值，初始为额定转速1435rpm）
				判断角位移传感器反馈的角度不大于30°。当大于30°时，转向电机转速为0
				Xavier向四合一发送55 AA AA 55 AA AA AA AA 逆变器1工作，转向电机风扇1工作
				*/
				cmd -> set_back_steering_target(10); //转向量为10
			}
			else
				back_steering_dir = 2; //后方转向电机不转
		}
		else{// 若出现异常
			motor_torque = pid_speed(0);
			front_steering_dir = 2;
			back_steering_dir = 2;
		}

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
