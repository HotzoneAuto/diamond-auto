#include <iostream>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/uart.h"
#include "modules/drivers/proto/rfid.pb.h"

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
  // TODO(wangying): auto config by udev
  Uart device_ = Uart("ttyUSB4");
  device_.SetOpt(9600, 8, 'N', 1);
  int count = 1;
  static char buffer[20];
  static char buf;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::RFID>> rfid_writer_ =
      node->CreateWriter<apollo::drivers::RFID>(FLAGS_rfid_topic);
  while (!apollo::cyber::IsShutdown()) {
    count = 0;
    std::memset(buffer, 0, 20);
    while (1) {
      int ret = device_.Read(&buf, 1);
//      AINFO << "RFID Device return" << ret;    
      if (ret == 1) {
        //AINFO << "RFID Device buf: " << buf;
        if (buf == 0x02) {
          count = 1;
          break;
        }
        buffer[count] = buf;
        count++;
      }
      if(count==10){
           AINFO << "RFID Device buf: " << buf;
           AINFO << "origin id from buffer[10]: " << buffer[10];
           uint32_t station_id = buffer[10] - '0';
           AINFO << "TRANSFER ID :" << station_id;
       }

      /*
      if (buf == 0x03 && count == 10) {
        AINFO << "origin id from buffer[10]: " << buffer[10];
       uint32_t station_id = buffer[10] - '0';
        AINFO << "TRANSFER ID :" << station_id;

        apollo::drivers::RFID rfid;
        auto header = rfid.mutable_header();
        header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
        header->set_frame_id("rfid");

        rfid.set_id(station_id);

        rfid_writer_->Write(rfid);
      }*/
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
