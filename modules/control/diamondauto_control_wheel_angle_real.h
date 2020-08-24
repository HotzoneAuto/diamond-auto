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
// ת�����
static float front_encoder_angle_realtime = 0; // ǰ�ֱ�����˲ʱ������0~360 deg��
static float front_encoder_angle_origin = 0; //ǰ�ֱ�����ԭʼ������0~32000��

static float rear_encoder_angle_realtime = 0; // ���ֱ�����˲ʱ������0~360 deg��
static float rear_encoder_angle_origin = 0; //���ֱ�����ԭʼ������0~32000��

static int front_wheel_steering_dir = 0; // ǰ������ת��0����ת��1��������ת��2��������ת
static int rear_wheel_steering_dir = 0; // ��������ת��0����ת��1��������ת��2��������ת

static int front_motor_steering_dir = 0; // ǰת��������ת����ת����̥��ת����ת����̥��ת����0����ֹͣ��1������ת��2����ת
static int rear_motor_steering_dir = 0; // ��ת��������ת����ת����̥��ת����ת����̥��ת����0����ֹͣ��1������ת��2����ת

static float front_encoder_angle_previous = 0; // ǰ�ֱ�������һʱ�̶�����0~360 deg��
static float rear_encoder_angle_previous = 0; // ǰ�ֱ�������һʱ�̶�����0~360 deg��

static const int encoder2wheel_gear_ratio = 125; // ����������̥���ٱ�

static float front_wheel_angle_previous = 0; // ��һʱ��ǰ��ת�ǣ�0~360 deg��
static float front_wheel_angle_realtime = 0; // ��ǰʱ��ǰ��ת�ǣ�0~360 deg��

static float rear_wheel_angle_previous = 0; // ��һʱ�̺���ת�ǣ�0~360 deg��
static float rear_wheel_angle_realtime = 0; // ��ǰʱ�̺���ת�ǣ�0~360 deg��


// �������
static int find_rfid_A = 0; // �Ƿ��⵽A���rfid��0�����1������
static int find_rfid_B = 0; // �Ƿ��⵽B���rfid��0�����1������

static int veh_dir = 0; // ��¼������ʻ����0�����A��B��1�����B��A

static int drivemotor_flag = 0; // �����������ת��־��0������ֹͣ��1��������ǰ�����������ת����2������������������ת��

static float drivemotor_torque = 0; //�������ת��

static const float speed_motor_deadzone_calibration = 50; // ���������ֹ״̬���𲽵������궨ֵ

static const float r_wheel = 0.34; // ���ְ뾶��m
static const float m_veh = 13000; // ����������kg
static const float g = 9.8; // �������ٶ�
static const float f_c = 0.018; // ����Ħ��ϵ��
static const float i_1 = 3.11; // ���ٱ�
static const float i_0 = 5.857; // �����ٱ�
static const float yita_t = 0.85; // ����Ч��




float update_wheel_angle(float wheel_angle_pre, float encoder_angle_pre, float encoder_angle_rt, const int encoder_to_wheel_gear_ratio)
{
	float delta_encoder_angle = encoder_angle_rt - encoder_angle_pre; // TODO: ����0��360��360��0ͻ�������

	// TODO: ��ֵ��ѡȡ
	if (delta_encoder_angle < -240) // ����������360��0��ͻ�䣬��̥����ת
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

	float wheel_angle_now = wheel_angle_pre + delta_encoder_angle / encoder_to_wheel_gear_ratio; // delta_encoder_angle������������������ת
	
	return wheel_angle_now;
}










}  // namespace control
}  // namespace apollo