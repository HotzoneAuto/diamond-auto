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

void OnData(std::shared_ptr<apollo::cyber::Node> node) {
  // TODO(all): config by udev or sudo usermod -aG dialout $USER
  Uart device_ = Uart("ttyUSB0");
  device_.SetOpt(9600, 8, 'N', 1);
  int count = 1;
  static char buffer[20];
  static char buf;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Magnetic>> magnetic_writer_ =
      node->CreateWriter<apollo::drivers::Magnetic>(FLAGS_magnetic_channel);
  while (!apollo::cyber::IsShutdown()) {
    count = 1;
    std::memset(buffer, 0, 20);
    while (1) {
      int ret = device_.Read(&buf, 1);
      AINFO << "Magnetic Device return: " << ret;

      if (ret == 1) {
        AINFO << "Magnetic RFID Device buf: " << buf;
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
        apollo::drivers::Magnetic magnetic;
        auto header = magnetic.mutable_header();
        header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
        header->set_frame_id("magnetic");

        AINFO << "RETURN ID : " << static_cast<int>(buffer[10]);

        // magnetic.set_id(static_cast<int>(buffer[10]));
        /*#include <iostream>
        using namespace std;
        int main()
        {
	        int count = 0;
	        static char * AGV_Buffer[16];
	        static char buf[2];
        	static char AGV_length;
		      while (1)
	          {
			        int c = count % 8;
			        AGV_Buffer[c] = buf;
			        count++;		
			          if (c == 3)
			          {
				           cout <<"第四位AGV_Buffer[3]是："<< AGV_Buffer[c] << endl;
			          }
			          if (c == 4)
		          	{
				           cout << "第五位AGV_Buffer[4]是：" << AGV_Buffer[c] << endl;
			          }
		         }	
		     return 0;
        }*/


        magnetic_writer_->Write(magnetic);
      }
    }
  }
}

int main(int32_t argc, char** argv) {
  apollo::cyber::Init(argv[0]);

  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  std::shared_ptr<apollo::cyber::Node> node = apollo::cyber::CreateNode("magnetic");
  OnData(node);
  // apollo::cyber::AsyncShutdown();
  // apollo::cyber::WaitForShutdown();
  return 0;
}
