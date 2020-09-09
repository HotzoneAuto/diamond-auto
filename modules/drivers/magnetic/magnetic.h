#pragma once

namespace apollo {
namespace drivers {
namespace magnetic {
// Steering const speed
unsigned char C1[8] = {0x0B, 0x06, 0x20, 0x00, 0x27, 0x10, 0x98, 0x9C};
// frontsteer stop
unsigned char C2[8] = {0x0B, 0x06, 0x06, 0x00, 0x00, 0x05, 0x4D, 0xA3};
// frontsteer positive
unsigned char C3[8] = {0x0B, 0x06, 0x10, 0x00, 0x00, 0x01, 0x4C, 0x60};
// frontsteer negative
unsigned char C4[8] = {0x0B, 0x06, 0x10, 0x00, 0x00, 0x02, 0x0C, 0x61};

// rear const speed
unsigned char C5[8] = {0x0C, 0x06, 0x20, 0x00, 0x27, 0x10, 0x99, 0x2B};
// rear stop
unsigned char C6[8] = {0x0C, 0x06, 0x10, 0x00, 0x00, 0x05, 0x4C, 0x14};
// rear positive
unsigned char C7[8] = {0x0C, 0x06, 0x10, 0x00, 0x00, 0x01, 0x4D, 0xD7};
// rear negative
unsigned char C8[8] = {0x0C, 0x06, 0x10, 0x00, 0x00, 0x02, 0x0D, 0xD6};

class Magnetic {
  //     public:
  // void Send(unsigned char cmd){
  //     int result_dir_positive = device_front_frequency->Write(cmd, 8);
  //   ADEBUG << "Frequency converter direction write command send result is :"
  //          << result_dir_positive;
  // }
  // private:
  //   std::unique_ptr<Uart> device_front_frequency = nullptr;
  //   std::unique_ptr<Uart> device_rear_frequency = nullptr;
};

}  // namespace magnetic
}  // namespace drivers
}  // namespace apollo
