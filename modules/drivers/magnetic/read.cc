#include <iostream>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/uart.h"
#include "modules/drivers/proto/magnetic.pb.h"

typedef unsigned char BYTE;

using namespace std;
using apollo::drivers::Magnetic;

char Hex2Ascii(char hex) {
  AINFO << "hex to transfer :" << hex;
  char c = toupper(hex);

  if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) {
    BYTE t = (c >= 'A') ? c - 'A' + 10 : c - '0';

    return t << 4;
  }
  return 'N';
}

// JUST DATA MODE
// TODO(wangying): need receive data with itensity
void OnData(std::shared_ptr<apollo::cyber::Node> node) {
  // TODO(wangying): auto config by udev
  Uart device_ = Uart("ttyUSB1");
  device_.SetOpt(9600, 8, 'N', 1);
  int count = 1;
  static char buffer[7];
  static char buf;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Magnetic>>
      magnetic_writer_ =
          node->CreateWriter<apollo::drivers::Magnetic>(FLAGS_magnetic_channel);
  while (!apollo::cyber::IsShutdown()) {
    // Send read Data message
    // char msg_read_cmd = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xd5, 0xca};
    char msg_read_cmd[8];
    msg_read_cmd[0] = 0x01;
    msg_read_cmd[1] = 0x03;
    msg_read_cmd[2] = 0x00;
    msg_read_cmd[3] = 0x01;
    msg_read_cmd[4] = 0x00;
    msg_read_cmd[5] = 0x01;
    msg_read_cmd[6] = 0xd5;
    msg_read_cmd[7] = 0xca;
    int result = device_.Write(msg_read_cmd, 8);
    ADEBUG << "Magnetic Msg Read Cmd Send result is :" << result;

    count = 1;
    std::memset(buffer, 0, 7);
    int ret = device_.Read(&buf, 1);
    AINFO << "Magnetic Device return: " << ret;

    if (ret == 1) {
      AINFO << "Magnetic Device buf: " << buf;
      buffer[count] = buf;
      count++;
      AINFO << "count: " << count;

      if (count == 7) {
        AINFO << "DEBUG READ OVER!!!!!!!!!!!!!";
        apollo::drivers::Magnetic magnetic;
        auto header = magnetic.mutable_header();
        header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
        header->set_frame_id("magnetic");

        AINFO << "RETURN ID buffer[3] : " << static_cast<int>(buffer[3]);
        AINFO << "RETURN ID buffer[4] : " << static_cast<int>(buffer[4]);

        // magnetic.set_id(static_cast<int>(buffer[10]));

        magnetic_writer_->Write(magnetic);
      }
    } else {
      AERROR << "MAGNETIC MSG READ ERROR "<< ret;
    }
  }
}

int main(int32_t argc, char** argv) {
  apollo::cyber::Init(argv[0]);

  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  std::shared_ptr<apollo::cyber::Node> node =
      apollo::cyber::CreateNode("magnetic");
  OnData(node);
  // apollo::cyber::AsyncShutdown();
  // apollo::cyber::WaitForShutdown();
  return 0;
}
