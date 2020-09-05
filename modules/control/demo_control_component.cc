#include "modules/control/demo_control_component.h"

#include <string>
#include "math.h"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/control_wheel_angle_real.h"
#include "modules/control/diamondauto_control_pid.h"

namespace apollo {
namespace control {

using apollo::cyber::Rate;
using apollo::cyber::Time;

bool ControlComponent::Init() {
  // Chassis Reader
  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
        chassis_.CopyFrom(*chassis);
      });
  /*
  // Magnetic Reader
  magnetic_reader_ = node_->CreateReader<Magnetic>(
      FLAGS_magnetic_channel,
      [this](const std::shared_ptr<Magnetic>& magnetic) {
        magnetic_.CopyFrom(*magnetic);
      });
  */
  // rfid Reader
  rfid_reader_ = node_->CreateReader<RFID>(
      FLAGS_rfid_topic,
      [this](const std::shared_ptr<RFID>& rfid) { rfid_.CopyFrom(*rfid); });

  // create Writer
  control_cmd_writer_ =
      node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);

  // compute control message in aysnc
  async_action_ = cyber::Async(&ControlComponent::GenerateCommand, this);

  return true;
}

// write to channel
void ControlComponent::GenerateCommand() {
  auto cmd = std::make_shared<ControlCommand>();

  // frequency
  Rate rate(100.0);

  // static int MANUAL_BRAKE_SIGNAL = 0;

  float front_lat_dev_mgs = 0.0;
  float rear_lat_dev_mgs = 0.0;

  // 获取当前车辆速度
  veh_spd = chassis_.speed_mps();

  /*
  // 获取前后编码器瞬时角度值
  if (isnan(chassis_.front_encoder_angle())){
    front_encoder_angle_realtime = front_encoder_angle_previous;
  }
  else{
    front_encoder_angle_realtime = chassis_.front_encoder_angle();
  }
  if (isnan(chassis_.rear_encoder_angle())){
    rear_encoder_angle_realtime = rear_encoder_angle_previous;
  }
  else{
    rear_encoder_angle_realtime = chassis_.rear_encoder_angle();
  }
  */

  // 初始化驱动电机死区
  if (veh_spd <= 0.1) {
    speed_motor_deadzone =
        speed_motor_deadzone_calibration;  // TODO: 落地后标定值
  } else {
    speed_motor_deadzone =
        r_wheel * m_veh * g * f_c /
        (i_1 * i_0 *
         yita_t);  // 使用滚动阻力系数，此时死区指的是理论电机需求转矩
  }

  // TODO:
  // 突然断电停车后，需记录最后时刻的状态数据，包括前后轮转角；重新启动后再读取数据文件

  // TODO: 在while判断前需要知道轮胎初始偏移角度，暂时先给0。
  // front_wheel_angle_realtime = 0;
  // rear_wheel_angle_realtime = 0;
  front_wheel_angle_realtime = chassis_.front_wheel_angle();
  rear_wheel_angle_realtime = chassis_.rear_wheel_angle();

  //上过高压自检完成之后，进入自动驾驶模式后，车辆处于ready状态时
  while (front_wheel_angle_realtime > 0.5)  // 待标定
  {
    // front_motor_steering_dir = 2;
    cmd->set_front_steering_switch(Chassis::STEERINGNEGATIVE);
    cmd->set_front_wheel_target(-10.0);

    // 获取编码器瞬时角度值
    // front_encoder_angle_realtime = chassis_.front_encoder_angle();

    // front_wheel_angle_realtime = chassis_.front_wheel_angle();

    /*
    front_wheel_angle_realtime = update_wheel_angle(
        front_wheel_angle_previous, front_encoder_angle_previous,
        front_encoder_angle_realtime, encoder2wheel_gear_ratio);

    front_wheel_angle_previous = front_wheel_angle_realtime;
    front_encoder_angle_previous = front_encoder_angle_realtime;
    */
  }

  while (rear_wheel_angle_realtime > 0.5)  // 待标定
  {
    // rear_motor_steering_dir = 2;  //则后方转向电机反转（即向左）
    cmd->set_rear_steering_switch(Chassis::STEERINGNEGATIVE);
    cmd->set_rear_wheel_target(-10.0);

    // 获取编码器瞬时角度值
    // rear_encoder_angle_realtime = chassis_.rear_encoder_angle();

    // rear_wheel_angle_realtime = chassis_.rear_wheel_angle();

    /*
    rear_wheel_angle_realtime = update_wheel_angle(
        rear_wheel_angle_previous, rear_encoder_angle_previous,
        rear_encoder_angle_realtime, encoder2wheel_gear_ratio);

    rear_wheel_angle_previous = rear_wheel_angle_realtime;
    rear_encoder_angle_previous = rear_encoder_angle_realtime;
    */
  }

  while (front_wheel_angle_realtime < -0.5)  // 待标定
  {
    // front_motor_steering_dir = 1;  //则前方转向电机正转（即向右）
    cmd->set_front_steering_switch(Chassis::STEERINGPOSITIVE);
    cmd->set_front_wheel_target(10.0);

    // 获取编码器瞬时角度值
    // front_encoder_angle_realtime = chassis_.front_encoder_angle();

    // front_wheel_angle_realtime = chassis_.front_wheel_angle();

    /*
    front_wheel_angle_realtime = update_wheel_angle(
        front_wheel_angle_previous, front_encoder_angle_previous,
        front_encoder_angle_realtime, encoder2wheel_gear_ratio);

    front_wheel_angle_previous = front_wheel_angle_realtime;
    front_encoder_angle_previous = front_encoder_angle_realtime;
    */
  }

  while (rear_wheel_angle_realtime < -0.5)  // 待标定
  {
    // rear_motor_steering_dir = 1;  //则后方转向电机正转（即向右）
    cmd->set_rear_steering_switch(Chassis::STEERINGPOSITIVE);
    cmd->set_rear_wheel_target(10.0);

    // 获取编码器瞬时角度值
    // rear_encoder_angle_realtime = chassis_.rear_encoder_angle();

    // rear_wheel_angle_realtime = chassis_.rear_wheel_angle();

    /*
    rear_wheel_angle_realtime = update_wheel_angle(
        rear_wheel_angle_previous, rear_encoder_angle_previous,
        rear_encoder_angle_realtime, encoder2wheel_gear_ratio);

    rear_wheel_angle_previous = rear_wheel_angle_realtime;
    rear_encoder_angle_previous = rear_encoder_angle_realtime;
    */
  }

  cmd->set_front_steering_switch(Chassis::STEERINGSTOP);
  // cmd->set_front_wheel_target(0.0);

  cmd->set_rear_steering_switch(Chassis::STEERINGSTOP);
  // cmd->set_rear_wheel_target(0.0);

  // front_motor_steering_dir = 0;  //则前方转向电机停转
  // rear_motor_steering_dir = 0;   //则后方转向电机停转

  while (true) {
    // TODO: Configuration
    // 手动给定，0代表停止，1代表从A到B，2代表从B到A
    drivemotor_flag = 1;
    switch (drivemotor_flag) {
      // 从A到B
      case 1: {
        if (rfid_.id() == 2) {
          // TODO: 制动转矩，需改成标定值
          drivemotor_torque = 50;
          cmd->set_front_brake(drivemotor_torque);
        } else {
          drivemotor_torque =
              pid_speed(veh_spd, FLAGS_desired_v, speed_motor_deadzone);
          cmd->set_front_throttle(drivemotor_torque);
        }

        // TODO：检测到车速为0，驻车系统断气刹，车辆驻车停止
        break;
      }

      // 从B到A
      case 2: {
        if (rfid_.id() == 1) {
          // TODO: 制动转矩，需改成标定值
          drivemotor_torque = 50;
          cmd->set_rear_brake(drivemotor_torque);
        } else {
          drivemotor_torque =
              pid_speed(veh_spd, FLAGS_desired_v, speed_motor_deadzone);
          cmd->set_rear_throttle(drivemotor_torque);
        }

        // TODO：检测到车速为0，驻车系统断气刹，车辆驻车停止
        break;
      }

      // 停止状态
      case 0: {
        cmd->set_front_throttle(0);
        cmd->set_rear_throttle(0);

        // TODO：检测到车速为0，驻车系统断气刹，车辆驻车停止
        break;
      }
    }

    /*
    // 遥控，人工给输入
    if (veh_dir == 0) //若行驶方向从A到B（前进，A是第2辆车处，B是机械臂处）
            drivemotor_flag = 1;
    else if (veh_dir == 1) //若行驶方向从B到A（后退）
            drivemotor_flag = 2;
    */

    /*
    front_wheel_angle_realtime = update_wheel_angle(
        front_wheel_angle_previous, front_encoder_angle_previous,
        front_encoder_angle_realtime, encoder2wheel_gear_ratio);
    rear_wheel_angle_realtime = update_wheel_angle(
        rear_wheel_angle_previous, rear_encoder_angle_previous,
        rear_encoder_angle_realtime, encoder2wheel_gear_ratio);
    */

    front_wheel_angle_realtime = chassis_.front_wheel_angle();
    rear_wheel_angle_realtime = chassis_.rear_wheel_angle();

    // 检测到轮胎转角超过30°，转向电机停转
    if (abs(front_wheel_angle_realtime) > 30) {
      // front_motor_steering_dir = 0;
      cmd->set_front_steering_switch(Chassis::STEERINGSTOP);
    }

    if (abs(rear_wheel_angle_realtime) > 30) {
      // rear_motor_steering_dir = 0;
      cmd->set_rear_steering_switch(Chassis::STEERINGSTOP);
    }

    /*
    // TODO：收到人工介入制动信号进行制动
    // TODO: manual_brake_signal:定义新通道专门用于自动控制下的人工交互
    MANUAL_BRAKE_SIGNAL = chassis_.manual_brake_signal();
    if ( MANUAL_BRAKE_SIGNAL != 0 ){
      // TODO: 制动转矩，需改成标定值
      drivemotor_torque = 50;
      cmd->set_brake(drivemotor_torque);
    }
    */

    switch (FLAGS_magnetic_enable) {
      case 1: {
        // 初始化前后磁导航检测到的偏差值，订阅磁导航通道的数据
        // TODO: 检查，共用了一个数据
        front_lat_dev_mgs = chassis_.front_lat_dev();
        rear_lat_dev_mgs = chassis_.rear_lat_dev();

        // 给定驱动电机反转命令（使车辆前进从A到B）
        if (drivemotor_flag == 1) {
          // rear_motor_steering_dir = 0;   //后方转向电机不转
          cmd->set_rear_steering_switch(Chassis::STEERINGSTOP);
          if (front_lat_dev_mgs < -4.5)  //若前方磁导航检测出车偏左
          {
            // front_motor_steering_dir = 1;  //则前方转向电机正转（即向右）
            cmd->set_front_steering_switch(Chassis::STEERINGPOSITIVE);
            cmd->set_front_wheel_target(10.0);
          } else if (front_lat_dev_mgs > 4.5)  //若前方磁导航检测出车偏右
          {
            // front_motor_steering_dir = 2;  //则前方转向电机反转（即向左）
            cmd->set_front_steering_switch(Chassis::STEERINGNEGATIVE);
            cmd->set_front_wheel_target(-10.0);
          } else {
            if (front_wheel_angle_realtime >= 0.5)  // 当前前轮转角为正，向右偏
            {
              // front_motor_steering_dir = 2;  // 前方转向电机反转（向左）
              cmd->set_front_steering_switch(Chassis::STEERINGNEGATIVE);
              cmd->set_front_wheel_target(0.0);
            } else if ((front_wheel_angle_realtime > -0.5) &&
                       (front_wheel_angle_realtime < 0.5)) {
              // front_motor_steering_dir = 0;  // 前方转向电机停转
              cmd->set_front_steering_switch(Chassis::STEERINGSTOP);
            } else  // 当前前轮转角为负，向左偏
            {
              // front_motor_steering_dir = 1;  // 前方转向电机正转（向右）
              cmd->set_front_steering_switch(Chassis::STEERINGPOSITIVE);
              cmd->set_front_wheel_target(0.0);
            }
          }
        } else if (drivemotor_flag == 2)  // 若驱动电机正转（倒车，车辆从B到A）
        {
          // front_motor_steering_dir = 0;  //前方转向电机不转
          cmd->set_front_steering_switch(Chassis::STEERINGSTOP);
          if (rear_lat_dev_mgs < -4.5)  //若后方磁导航检测出车偏左
          {
            // rear_motor_steering_dir = 1;  //则后方转向电机正转（即向右）
            cmd->set_rear_steering_switch(Chassis::STEERINGPOSITIVE);
            cmd->set_rear_wheel_target(10.0);
          } else if (rear_lat_dev_mgs > 4.5)  //若后方磁导航检测出车偏右
          {
            // rear_motor_steering_dir = 2;  //则后方转向电机反转（即向左）
            cmd->set_rear_steering_switch(Chassis::STEERINGNEGATIVE);
            cmd->set_rear_wheel_target(-10.0);
          } else {
            if (rear_wheel_angle_realtime >= 0.5)  // 当前后轮转角为正，向右偏
            {
              // rear_motor_steering_dir = 2;  // 后方转向电机反转（向左）
              cmd->set_rear_steering_switch(Chassis::STEERINGNEGATIVE);
              cmd->set_rear_wheel_target(0);
            } else if ((rear_wheel_angle_realtime > -0.5) &&
                       (rear_wheel_angle_realtime < 0.5)) {
              // rear_motor_steering_dir = 0;  // 后方转向电机停转
              cmd->set_rear_steering_switch(Chassis::STEERINGSTOP);
            } else  // 当前后轮转角为负，向左偏
            {
              // rear_motor_steering_dir = 1;  // 后方转向电机正转（向右）
              cmd->set_rear_steering_switch(Chassis::STEERINGPOSITIVE);
              cmd->set_rear_wheel_target(0);
            }
          }
        } else {  // 若出现异常
          // auto motor_torque = pid_speed(veh_spd, 0, speed_motor_deadzone);
          // front_motor_steering_dir = 0;  // 停止
          // rear_motor_steering_dir = 0;   // 停止
          cmd->set_front_steering_switch(Chassis::STEERINGSTOP);
          cmd->set_rear_steering_switch(Chassis::STEERINGSTOP);
        }
        break;
      }
      case 0: {
        if (drivemotor_flag == 1) {
          // rear_motor_steering_dir = 0;
          cmd->set_rear_steering_switch(Chassis::STEERINGSTOP);
          cmd->set_front_steering_switch(manual_front_steering_switch);
          cmd->set_front_wheel_target(manual_front_wheel_target);
        } else if (drivemotor_flag == 2) {
          // front_motor_steering_dir = 0;
          cmd->set_front_steering_switch(Chassis::STEERINGSTOP);
          cmd->set_rear_steering_switch(manual_rear_steering_switch);
          cmd->set_rear_wheel_target(manual_rear_wheel_target);
        } else {
          // front_motor_steering_dir = 0;
          // rear_motor_steering_dir = 0;
          cmd->set_front_steering_switch(Chassis::STEERINGSTOP);
          cmd->set_rear_steering_switch(Chassis::STEERINGSTOP);
        }
        break;
      }
      default: {}
    }

    /*
    // 更新轮胎转角和编码器度数
    front_wheel_angle_realtime = update_wheel_angle(
        front_wheel_angle_previous, front_encoder_angle_previous,
        front_encoder_angle_realtime, encoder2wheel_gear_ratio);
    rear_wheel_angle_realtime = update_wheel_angle(
        rear_wheel_angle_previous, rear_encoder_angle_previous,
        rear_encoder_angle_realtime, encoder2wheel_gear_ratio);

    front_wheel_angle_previous = front_wheel_angle_realtime;
    rear_wheel_angle_previous = rear_wheel_angle_realtime;

    front_encoder_angle_previous = front_encoder_angle_realtime;
    rear_encoder_angle_previous = rear_encoder_angle_realtime;

    chassis_wheel_angle->set_front_wheel_angle(front_wheel_angle_realtime);
    chassis_wheel_angle->set_rear_wheel_angle(rear_wheel_angle_realtime);

    chassis_writer_->Write(chassis_wheel_angle);
    */

    control_cmd_writer_->Write(cmd);

    rate.Sleep();
  }
}

ControlComponent::~ControlComponent() {
  // back chassis handle
  async_action_.wait();
}

}  // namespace control
}  // namespace apollo
