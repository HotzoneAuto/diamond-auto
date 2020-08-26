#pragma once

#include <stdlib.h>
#include <cmath>
#include <iostream>

namespace apollo
{
namespace control
{

using namespace std;

// global
// 转向相关
static float front_encoder_angle_realtime = 0; // 前轮编码器瞬时度数（0~360 deg）
static float front_encoder_angle_origin = 0; //前轮编码器原始读数（0~32000）

static float rear_encoder_angle_realtime = 0; // 后轮编码器瞬时度数（0~360 deg）
static float rear_encoder_angle_origin = 0; //后轮编码器原始读数（0~32000）

static int front_wheel_steering_dir = 0; // 前轮左右转，0代表不转，1代表向右转，2代表向左转
static int rear_wheel_steering_dir = 0; // 后轮左右转，0代表不转，1代表向右转，2代表向左转

static int front_motor_steering_dir = 0; // 前转向电机正反转（正转：轮胎右转；反转：轮胎左转），0代表停止，1代表正转，2代表反转
static int rear_motor_steering_dir = 0; // 后转向电机正反转（正转：轮胎右转；反转：轮胎左转），0代表停止，1代表正转，2代表反转

static float front_encoder_angle_previous = 0; // 前轮编码器上一时刻度数（0~360 deg）
static float rear_encoder_angle_previous = 0; // 前轮编码器上一时刻度数（0~360 deg）

static const int encoder2wheel_gear_ratio = 125; // 编码器：轮胎的速比

static float front_wheel_angle_previous = 0; // 上一时刻前轮转角（0~360 deg）
static float front_wheel_angle_realtime = 0; // 当前时刻前轮转角（0~360 deg）

static float rear_wheel_angle_previous = 0; // 上一时刻后轮转角（0~360 deg）
static float rear_wheel_angle_realtime = 0; // 当前时刻后轮转角（0~360 deg）


// 驱动相关
static int find_rfid_A = 0; // 是否检测到A点的rfid，0代表否，1代表是
static int find_rfid_B = 0; // 是否检测到B点的rfid，0代表否，1代表是

static int veh_dir = 0; // 记录车辆行驶方向，0代表从A到B，1代表从B到A

static int drivemotor_flag = 0; // 驱动电机正反转标志，0代表车辆停止，1代表车辆向前（驱动电机反转），2代表车辆向后（驱动电机正转）

static float drivemotor_torque = 0; //驱动电机转矩

static const float speed_motor_deadzone_calibration = 50; // 驱动电机静止状态下起步的死区标定值

static const float r_wheel = 0.34; // 车轮半径，m
static const float m_veh = 13000; // 整车质量，kg
static const float g = 9.8; // 重力加速度
static const float f_c = 0.018; // 滚动摩擦系数
static const float i_1 = 3.11; // 减速比
static const float i_0 = 5.857; // 主减速比
static const float yita_t = 0.85; // 传动效率




float update_wheel_angle(float wheel_angle_pre, float encoder_angle_pre, float encoder_angle_rt, const int encoder_to_wheel_gear_ratio)
{
	float delta_encoder_angle = encoder_angle_rt - encoder_angle_pre; // TODO: 需解决0到360、360到0突变的问题

	// TODO: 阈值的选取
	if (delta_encoder_angle < -240) // 编码器发生360到0的突变，轮胎向右转
	{
		delta_encoder_angle = delta_encoder_angle + 360;
	}
	else if (delta_encoder_angle > 240) // 
	{
		delta_encoder_angle = delta_encoder_angle - 360;
	}
	else
	{
		delta_encoder_angle = delta_encoder_angle;
	}

	float wheel_angle_now = wheel_angle_pre + delta_encoder_angle / encoder_to_wheel_gear_ratio; // delta_encoder_angle有正负，包含了左右转
	
	return wheel_angle_now;
}










}  // namespace control
}  // namespace apollo