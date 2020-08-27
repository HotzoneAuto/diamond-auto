#pragma once

#include <stdlib.h>
#include <cmath>
#include <iostream>


namespace apollo
{
namespace control
{

using namespace std;

// static float kp_steer = 0.1;
// static float ki_steer = 0;
// static float kd_steer = 0.1;
// static float delta_u_steer;
// static float u_pre_steer = 0;
// static float u_steer;
// static int steer_direction;

static float kp_speed = 0;
static float ki_speed = 0;
static float kd_speed = 0;
// static float delta_u_torque;
// static float u_pre_torque = 0;
static float u_torque;

// static float steer_motor_spd;
// static float tire_steer_spd;
// static float steer_angle=0;
static float veh_spd = 0;

static float speed_motor_deadzone = 0; 

static float pid_integral = 0;
static float pid_error = 0;
static float pid_error_pre = 0;
// static float fmottq = 0;
// static float canbus_veh_spd = 0;
// static int veh_mode = 0;


float pid_speed (float veh_spd,float desire_v,float spd_motor_deadzone) 
{
	cout << "pid中的期望速度" << desire_v << endl;
	// veh_spd = chassis_->speed_mps();
	pid_error = desire_v - veh_spd;  // pid输入为当前车速误差
	cout << "纵向速度偏差" << pid_error << endl;

	pid_integral += pid_error;
	
	u_torque = spd_motor_deadzone + kp_speed * pid_error +
	           ki_speed * pid_integral + kd_speed * (pid_error - pid_error_pre);

	cout << "PID输出给驱动电机的控制量（转矩）：" << u_torque << endl;

	// veh_spd = canbus_veh_spd;

	cout << "当前车速" << veh_spd << endl;
	pid_error_pre = pid_error;

	return u_torque;

	// 将u_torque通过CAN通讯发送给驱动电机

	// u_pre_torque = u_torque;

	// 将CANbus反馈的车速付给当前车速
}

}  // namespace control
}  // namespace apollo
