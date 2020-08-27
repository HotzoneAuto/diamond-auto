
#include "modules/drivers/magnetic/magnetic_component.h"

#include <string>

#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace drivers {
namespace magnetic {

int decToBin(int dec) {
  int result = 0, temp = dec, j = 1;
  while (temp) {
    result = result + j * (temp % 2);
    temp = temp / 2;
    j = j * 10;
  }
  return result;
}

bool MagneticComponent::Init() {
  // Uart device set option
  device_.SetOpt(9600, 8, 'N', 1);

  // Publish rfid station data
  magnetic_writer_ = node_->CreateWriter<Magnetic>(FLAGS_magnetic_channel);

  // Async read
  async_action_ = cyber::Async(&MagneticComponent::Action, this);

  return true;
}

// TODO()CHECK
bool MagneticComponent::Check() { return true; }

void MagneticComponent::Action() {
  int count_front = 1;
  static char buffer_front[7];
  static char buf_front;
  int count_rear = 1;
  static char buffer_rear[7];
  static char buf_rear;

  while (!apollo::cyber::IsShutdown()) {
    // Send whrit Data message
    // char msg_read_cmd = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xd5, 0xca};
    char front_msg_write_cmd[8];
    front_msg_write_cmd[0] = 0x01;
    front_msg_write_cmd[1] = 0x03;
    front_msg_write_cmd[2] = 0x00;
    front_msg_write_cmd[3] = 0x01;
    front_msg_write_cmd[4] = 0x00;
    front_msg_write_cmd[5] = 0x01;
    front_msg_write_cmd[6] = 0xd5;
    front_msg_write_cmd[7] = 0xca;

    int result_front = device_mgs_front.Write(front_msg_write_cmd, 8);
    ADEBUG << "Magnetic Msg Read Cmd Send result is :" << result_front;

    count_front = 1;
    std::memset(buffer_front, 0, 10);
    while (1) {
      int ret_front = device_.Read(&buf_front, 1);
      if (ret_front == 1) {
        if (buf_front == 0x01) {
          break;
        }
        buffer_front[count_front] = buf_front;
        count_front++;
      }
    }

    if (count_front == 7) {
      apollo::drivers::Magnetic magnetic;
      auto header = magnetic.mutable_header();
      header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
      header->set_frame_id("magnetic");

      AINFO << "RETURN ID buffer_front[3] : "
            << static_cast<int>(buffer_front[3]);  // low 8
      AINFO << "RETURN ID buffer_front[4] : "
            << static_cast<int>(buffer_front[4]);  // high 8

      std::string s_l_front = std::to_string(decToBin(buffer_front[4]));
      std::string s_h_front = std::to_string(decToBin(buffer_front[3]));

      while (s_l_front.size() < 8) {
        s_l_front = '0' + s_l_front;
      }

      while (s_h_front.size() < 8) {
        s_h_front = '0' + s_h_front;
      }

      char ss_front[16] = {0};
      for (int i = 0; i < 16;
           i++)  // convert 0000 0001 0000 0000 to 0000 0000 1000 0000, make
                 // string each char in ss denote 1~16 in sequence.
      {
        if (i < 8) {
          ss_front[i] = s_l_front[7 - i];
        } else
          ss_front[i] = s_h_front[15 - i];
      }

      int sum_activated_front = 0;
      int sum_id_front = 0;
      for (int i = 0; i < 16; i++) {
        if (ss_front[i] == '1') {
          sum_id_front += i + 1;
          sum_activated_front += 1;
        }
      }

      double front_lat_dev_mgs = 0;
      front_lat_dev_mgs = double(sum_id_front) / double(sum_activated_front) - 8.5;

      magnetic.set_front_lat_dev(static_cast<float>(front_lat_dev_mgs));

      magnetic_writer_->Write(magnetic);
    }


	// Send whrit Data message
    // char msg_read_cmd = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xd5, 0xca};
    char rear_msg_write_cmd[8];
    rear_msg_write_cmd[0] = 0x01;
    rear_msg_write_cmd[1] = 0x03;
    rear_msg_write_cmd[2] = 0x00;
    rear_msg_write_cmd[3] = 0x01;
    rear_msg_write_cmd[4] = 0x00;
    rear_msg_write_cmd[5] = 0x01;
    rear_msg_write_cmd[6] = 0xd5;
    rear_msg_write_cmd[7] = 0xca;

    int result_rear = device_mgs_rear.Write(rear_msg_write_cmd, 8);
    ADEBUG << "Magnetic Msg Read Cmd Send result is :" << result_rear;

    count_rear = 1;
    std::memset(buffer_rear, 0, 10);
    while (1) {
      int ret_rear = device_.Read(&buf_rear, 1);
      if (ret_rear == 1) {
        if (buf_rear == 0x01) {
          break;
        }
        buffer_rear[count_rear] = buf_rear;
        count_rear++;
      }
    }

    if (count_rear == 7) {
      // apollo::drivers::Magnetic magnetic_rear;
      // auto header_rear = magnetic_rear.mutable_header();
      // header_rear->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
      // header_rear->set_frame_id("magnetic");

      AINFO << "RETURN ID buffer_rear[3] : "
            << static_cast<int>(buffer_rear[3]);  // low 8
      AINFO << "RETURN ID buffer_rear[4] : "
            << static_cast<int>(buffer_rear[4]);  // high 8

      std::string s_l_rear = std::to_string(decToBin(buffer_rear[4]));
      std::string s_h_rear = std::to_string(decToBin(buffer_rear[3]));

      while (s_l_rear.size() < 8) {
        s_l_rear = '0' + s_l_rear;
      }

      while (s_h_rear.size() < 8) {
        s_h_rear = '0' + s_h_rear;
      }

      char ss_rear[16] = {0};
      for (int i = 0; i < 16;
           i++)  // convert 0000 0001 0000 0000 to 0000 0000 1000 0000, make
                 // string each char in ss denote 1~16 in sequence.
      {
        if (i < 8) {
          ss_rear[i] = s_l_rear[7 - i];
        } else
          ss_rear[i] = s_h_rear[15 - i];
      }

      int sum_activated_rear = 0;
      int sum_id_rear = 0;
      for (int i = 0; i < 16; i++) {
        if (ss_rear[i] == '1') {
          sum_id_rear += i + 1;
          sum_activated_rear += 1;
        }
      }

      double rear_lat_dev_mgs = 0;
      rear_lat_dev_mgs = double(sum_id_rear) / double(sum_activated_rear) - 8.5;

      magnetic.set_rear_lat_dev(static_cast<float>(rear_lat_dev_mgs));

      magnetic_writer_->Write(magnetic);
    }

  }
}

MagneticComponent::~MagneticComponent() {
  AINFO << "MagneticComponent::~MagneticComponent()";
}

}  // namespace magnetic
}  // namespace drivers
}  // namespace apollo
