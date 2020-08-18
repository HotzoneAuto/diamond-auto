#pragma once

#include <stdlib.h>
#include <cmath>
#include <iostream>

namespace apollo {
namespace control {

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

static float pid_integral = 0;  // ����pid�ۻ���
static float pid_error = 0;
static float pid_error_pre = 0;
// static float fmottq = 0;
static float canbus_veh_spd = 0;
// static int veh_mode = 0;

// void pid_steering(float & e2, float & e1, float & e); //
// ת��PID���ƣ�PID�����������ת�٣���ת���� float steer_motor(float error);
// // ���������Ӧ�������˶��������һʱ�̵Ĵŵ���ƫ�� void pid_speed(float
// desire_v); // �����ٶ�PID���ƣ�PID�����������ת�أ����������

void Init() { cout << "��ʼ��......" << endl << endl; };

void Self_check() { cout << "����״̬�Լ�......" << endl << endl; };

/*void pid_steering(float & e2, float & e1, float & e)
{
        cout<<"����ǰ�Ĵŵ���ƫ�"<<e2<<"��"<<e1<<"��"<<e<<endl;

        delta_u_steer = kp_steer*(e-e1)+ki_steer*(e)+kd_steer*(e-2*e1+e2); //
pid����Ŀ���������

        u_steer = u_pre_steer + delta_u_steer;

        cout<<"PID�����ת�����Ŀ�������"<<u_steer<<endl;

        u_pre_steer = u_steer;

        // �жϵ������ת
        if (e>0)
        {
                cout<<"ת����Ӧ��ת"<<endl;
                steer_direction = 1;
        }
        else if (e<0)
        {
                cout<<"ת����Ӧ��ת"<<endl;
                steer_direction = -1;
        }
        else
        {
                cout<<"ת����Ӧֱ��"<<endl;
                steer_direction = 0;
        }

        e2 = e1;
        e1 = e;
        e = steer_motor(e);

        cout<<"����PID���ڣ��µĴŵ���ƫ�����У�"<<e2<<"��"<<e1<<"��"<<e<<endl;

};
*/

/* float steer_motor(float e)
{
        if (steer_direction == 1) // ��ת
        {
                cout<<"�����ת"<<endl;
                // CANͨѶ��ת������ת���
        }
        else if(steer_direction == -1) // ��ת
        {
                cout<<"�����ת"<<endl;
                // CANͨѶ��ת������ת���
        }


        // ���ģ�ͣ��� pid ������ �� ת�������ת�٣�
        steer_motor_spd = u_steer;// ���д��


        // ���ٻ�����ת�������ת�� ͨ�����ٻ��� �� ��̥ת���ٶȣ�
        tire_steer_spd = steer_motor_spd/20;


        // �˶����£���̥ת���ٶ� ��
��̥ת��(��λ�ƴ�����)�������������˶���״̬���� steer_angle = steer_angle +
tire_steer_spd;//��λ�ƴ���������


        // �µ�ƫ���⣺�ŵ�����������⵱ǰƫ��
        cout<<"��̥ת�ǣ�"<<steer_angle<<endl;

        float error_new;
        error_new = e+sin(steer_angle*3.1415926/180)*veh_spd*100*0.05;
//0.05s����һ��
        // cout<<"�µ�ƫ�"<<error_new<<endl;
        return error_new;
};
*/

void simple_steering(float& e) {
  if (e > 4.5 || e < -4.5) {
    // �·���ת������ʼת���ź�
    cout << "ת������ʼת" << endl;
  } else {
    // �·���ת����ֹͣת���ź�
    cout << "ת����ֹͣת" << endl;
  }
}

void rule_steering(float& e) {
  // �ж�ǰ��ת��������ת
  if (e > 4.5) {
    cout << "ת����Ӧ��ת" << endl;  //����ƫ��ģ�鷴����ֵ���е���
    steer_direction = 1;
  } else if (e < -4.5) {
    cout << "ת����Ӧ��ת" << endl;
    steer_direction = 2;
  } else {
    cout << "ת����Ӧֱ��" << endl;
    steer_direction = 0;
  }

  // ������ת�źŷ���CANͨѶ

  // ǰ��ת��������CAN��������ת�ź�

  if (steer_direction == 1)  // ��ת
  {
    cout << "�����ת" << endl;
    // CANͨѶ��ת������ת���

    if (e >= 6) {
      steer_motor_spd = 1435;  // ���ݾ����������Ҫ��
    } else if (e >= 3) {
      steer_motor_spd = 1435;
    } else {
      steer_motor_spd = 1435;
    }
  } else if (steer_direction == 2)  // ��ת
  {
    cout << "�����ת" << endl;
    // CANͨѶ��ת������ת���

    if (e <= -6) {
      steer_motor_spd =
          1435;  // ת�����ת��1435rpm��Ŀǰ���ɿأ����ڳ��Կɿ��У��ת������70sת1Ȧ������0.857rpm
    } else if (e <= -3) {
      steer_motor_spd = 1435;
    } else {
      steer_motor_spd = 1435;
    }
  }

  cout << "ת�ٵ��ת��Ϊ" << steer_motor_spd << endl;

  e = e / 2;  // �˴�Ӧ���Ǵŵ���������ʵʱ���յ���ƫ��

  cout << "�µĴŵ���ƫ��Ϊ��" << e << endl;
}

float pid_speed(float desire_v)  // Ŀ������pid���ת��
{
  cout << "pid�е������ٶȣ�" << desire_v << endl;
  pid_error = desire_v - veh_spd;  // pid����Ϊ��ǰ�������
  cout << "�����ٶ�ƫ�" << pid_error << endl;

  pid_integral += pid_error;

  // speed_motor_errorӦ����һ���������ٶ���ص�ǰ���������������ٶȿ������һ�����ٶ������µ�Ŀ��ת��ֵ
  // speed_motor_errorӦ��������Ŀ��ת��
  u_torque = speed_motor_deadzone + kp_speed * pid_error +
             ki_speed * pid_integral + kd_speed * (pid_error - pid_error_pre);

  cout << "PID�������������Ŀ�����(ת��)��" << u_torque << endl;

  veh_spd =
      canbus_veh_spd;  // ���ٸ��£���Ҫ���ݸ���������Ŀ����������������Ӧ��

  cout << "��ǰ���٣�" << veh_spd << endl;
  pid_error_pre = pid_error;

  return u_torque;

  // ��u_torqueͨ��CANͨѶ���͸��������

  // u_pre_torque = u_torque;

  // ��CANbus�����ĳ��ٸ�����ǰ����
}

}  // namespace control
}  // namespace apollo
