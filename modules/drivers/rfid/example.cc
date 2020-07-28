#include <iostream>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"

#include "modules/common/util/uart.h"
#include "modules/drivers/proto/rfid.pb.h"
#include "modules/common/adapters/adapter_gflags.h"

typedef unsigned char BYTE;

using namespace std;

char Hex2Ascii(char hex) {
  AINFO << "hex to transfer :" << hex;
  char c = toupper(hex);

  if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) {
    BYTE t = (c >= 'A') ? c - 'A' + 10 : c - '0';

    return t << 4;
  }
  return 'N';
}

void OnData(std::shared_ptr<apollo::cyber::Node> node) {
  // TODO(all): config by udev or sudo usermod -aG dialout $USER
  Uart device_ = Uart("ttyUSB0");
  device_.SetOpt(9600, 8, 'N', 1);
  int count = 1;
  static char buffer[20];
  static char buf;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::RFID>> rfid_writer_ = 
        node->CreateWriter<apollo::drivers::RFID>(FLAGS_rfid_topic);
  while (!apollo::cyber::IsShutdown()) {
    count = 1;
    std::memset(buffer, 0, 20);
    while (1) {
      int ret = device_.Read(&buf, 1);
      AINFO << "RFID Device return: " << ret;

      if (ret == 1) {
        AINFO << "RFID Device buf: " << buf;
        if (buf == 0x02) {
          count = 1;
          break;
        }
        buffer[count] = buf;
        count++;
      }
      AINFO << "count: " << count;
          if (count == 13) {
        AINFO << "DEBUG READ OVER!!!!!!!!!!!!!";
        // for(auto & b : buffer) {
        // auto as = Hex2Ascii(b);
        // AINFO << "retrun transfered: " << as;
        // }
        // auto a = Hex2Ascii(buffer[10]);
        // AINFO << "CARD ID :" << static_cast<int>(a);

        apollo::drivers::RFID rfid;
        auto header = rfid.mutable_header();
        header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
        header->set_frame_id("rfid");

        AINFO << "CARD ID : " << static_cast<int>(buffer[10]);
        
        rfid.set_id(static_cast<int>(buffer[10]));

        rfid_writer_->Write(rfid);
      }
    }


    
  }
}

int main(int32_t argc, char** argv) {
  apollo::cyber::Init(argv[0]);

  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  std::shared_ptr<apollo::cyber::Node> node = apollo::cyber::CreateNode("rfid");
  OnData(node);
  // apollo::cyber::AsyncShutdown();
  // apollo::cyber::WaitForShutdown();
  return 0;
}
