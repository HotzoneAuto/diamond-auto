#ifndef _DIAMONDAUTO_CONTROL_H_
#define _DIAMONDAUTO_CONTROL_H_

#include <stdlib.h>
#include <iostream>
#include <cmath>

using namespace std;

// static float kp_steer = 0.1;
// static float ki_steer = 0;
// static float kd_steer = 0.1;
// static float delta_u_steer;
// static float u_pre_steer = 0;
// static float u_steer;
static int steer_direction;

static float kp_speed = 0;
static float ki_speed = 0;
static float kd_speed = 0;
// static float delta_u_torque;
// static float u_pre_torque = 0;
static float u_torque;

static float steer_motor_spd;
// static float tire_steer_spd;
// static float steer_angle=0;
static float veh_spd = 0;
// static float desired_v = 2; 

static float speed_motor_deadzone = 0;

static float pid_integral = 0; // 纵向pid累积量
static float pid_error = 0; 
static float pid_error_pre = 0;
// static float fmottq = 0;
static float canbus_veh_spd = 0;
// static int veh_mode = 0;

// void pid_steering(float & e2, float & e1, float & e); // 转向PID控制，PID输出控制量（转速）给转向电机
// float steer_motor(float error); // 经过电机响应、车辆运动，输出下一时刻的磁导航偏差
// void pid_speed(float desire_v); // 纵向速度PID控制，PID输出控制量（转矩）给驱动电机

void Init()
{
	cout<<"初始化......"<<endl<<endl;
};

void Self_check()
{
	cout<<"车辆状态自检......"<<endl<<endl;
};


/*void pid_steering(float & e2, float & e1, float & e)
{
	cout<<"控制前的磁导航偏差："<<e2<<"、"<<e1<<"、"<<e<<endl;

	delta_u_steer = kp_steer*(e-e1)+ki_steer*(e)+kd_steer*(e-2*e1+e2); // pid输出的控制量增量
	
	u_steer = u_pre_steer + delta_u_steer;

	cout<<"PID输出给转向电机的控制量："<<u_steer<<endl;

	u_pre_steer = u_steer;

	// 判断电机正反转
	if (e>0)
	{
		cout<<"转向轮应左转"<<endl;
		steer_direction = 1; 
	}
	else if (e<0)
	{
		cout<<"转向轮应右转"<<endl;
		steer_direction = -1;
	}
	else
	{
		cout<<"转向轮应直行"<<endl;
		steer_direction = 0;
	}

	e2 = e1;
	e1 = e;
	e = steer_motor(e);
	
	cout<<"经过PID调节，新的磁导航偏差序列："<<e2<<"、"<<e1<<"、"<<e<<endl;
			
};
*/

/* float steer_motor(float e)
{
	if (steer_direction == 1) // 左转
	{
		cout<<"电机左转"<<endl;
		// CAN通讯给转向电机左转命令；		
	}
	else if(steer_direction == -1) // 右转
	{
		cout<<"电机右转"<<endl;
		// CAN通讯给转向电机右转命令；		
	}
	

	// 电机模型：由 pid 控制量 到 转向电机输出转速；
	steer_motor_spd = u_steer;// 随便写的


	// 减速机构：转向电机输出转速 通过减速机构 到 轮胎转向速度；
	tire_steer_spd = steer_motor_spd/20;
	

	// 运动更新：轮胎转向速度 到 轮胎转角(角位移传感器)；车辆横纵向运动，状态更新
	steer_angle = steer_angle + tire_steer_spd;//角位移传感器测量
	

	// 新的偏差检测：磁导航传感器检测当前偏差
	cout<<"轮胎转角："<<steer_angle<<endl;
		
	float error_new;
	error_new = e+sin(steer_angle*3.1415926/180)*veh_spd*100*0.05; //0.05s控制一次
	// cout<<"新的偏差："<<error_new<<endl;
	return error_new;
};
*/

void simple_steering(float & e)
{
	if (e>4.5 || e<-4.5)
	{
		// 下发让转向电机开始转的信号
		cout<<"转向电机开始转"<<endl;
	}
	else 
	{
		// 下发让转向电机停止转的信号	
		cout<<"转向电机停止转"<<endl;
	}

}


void rule_steering(float & e)
{
	// 判断前面转向电机正反转
	if (e>4.5)
	{
		cout<<"转向轮应左转"<<endl; //根据偏差模块反馈的值进行调整
		steer_direction = 1; 
	}
	else if (e<-4.5)
	{
		cout<<"转向轮应右转"<<endl;
		steer_direction = 2;
	}
	else
	{
		cout<<"转向轮应直行"<<endl;
		steer_direction = 0;
	}

	// 把正反转信号发给CAN通讯

	// 前面转向电机接收CAN给的正反转信号

	if (steer_direction == 1) // 左转
	{
		cout<<"电机左转"<<endl;
		// CAN通讯给转向电机左转命令；	

		if (e>=6)
		{
			steer_motor_spd = 1435; // 数据具体给多少需要算
		}
		else if (e>=3)
		{
			steer_motor_spd = 1435;	
		}
		else
		{
			steer_motor_spd = 1435;	
		}
	}
	else if(steer_direction == 2) // 右转
	{
		cout<<"电机右转"<<endl;
		// CAN通讯给转向电机右转命令；	

		if (e<=-6)
		{
			steer_motor_spd = 1435;	// 转向电机额定转速1435rpm，目前不可控，正在尝试可控中，额定转速下是70s转1圈，车轮0.857rpm
		}
		else if (e<=-3)
		{
			steer_motor_spd = 1435;	
		}
		else
		{
			steer_motor_spd = 1435;	
		}
	}

	cout<<"转速电机转速为"<<steer_motor_spd<<endl;
	
	e = e/2; // 此处应该是磁导航传感器实时接收的新偏差

	cout<<"新的磁导航偏差为："<<e<<endl;
}

float pid_speed(float desire_v) // 目的是用pid求出转矩
{
	cout<<"pid中的期望速度："<<desire_v<<endl;
	pid_error = desire_v - veh_spd;  // pid输入为当前车速误差
	cout<< "纵向速度偏差："<<pid_error<<endl;

	pid_integral += pid_error;
	
	// speed_motor_error应该是一个与期望速度相关的前馈量，给定期望速度可以算出一个该速度匀速下的目标转矩值
	// speed_motor_error应该是整车目标转矩
	u_torque = speed_motor_deadzone + kp_speed*pid_error + ki_speed*pid_integral + kd_speed*(pid_error-pid_error_pre);

	cout<<"PID输出给驱动电机的控制量(转矩)："<<u_torque<<endl;

	veh_spd = canbus_veh_spd; // 车速更新；需要根据给驱动电机的控制量计算出车速响应；	
	
	cout<<"当前车速："<<veh_spd<<endl;
	pid_error_pre  = pid_error;	 
	
	return u_torque;
	
	// 将u_torque通过CAN通讯发送给驱动电机

	// u_pre_torque = u_torque;

	// 将CANbus反馈的车速赋给当前车速
}


#endif