#include <iostream>
#include <string.h>

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

int decToBin(int dec){
	int result = 0, temp = dec, j = 1;
	while(temp){
		result = result + j * (temp % 2);
		temp = temp / 2;
		j = j * 10;
	}
	return result;
}


// JUST DATA MODE
// TODO(wangying): need receive data with itensity
void OnData(std::shared_ptr<apollo::cyber::Node> node) {
  // TODO(wangying): auto config by udev
  Uart device_ = Uart("ttyUSB0");
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
    std::memset(buffer, 0, 10);
    while (1) {
      int ret = device_.Read(&buf, 1);
      if (ret == 1) {
        if (buf == 0x01) {
          break;
        }
        buffer[count] = buf;
        count++;
      }
    }

    if (count == 7) {
      apollo::drivers::Magnetic magnetic;
      auto header = magnetic.mutable_header();
      header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
      header->set_frame_id("magnetic");

      AINFO << "RETURN ID buffer[3] : " << static_cast<int>(buffer[3]); // low 8
      AINFO << "RETURN ID buffer[4] : " << static_cast<int>(buffer[4]); // high 8

	  string s_l = to_string(decToBin(buffer[4]));
	  string s_h = to_string(decToBin(buffer[3]));

	  while (s_l.size()<8)
	  {
		s_l = '0'+s_l;		
	  }

	  while (s_h.size()<8)
	  {
		s_h = '0'+s_h;		
	  }
	  
	  char ss[16]={0};
	  for(i=0;i<16;i++) // convert 0000 0001 0000 0000 to 0000 0000 1000 0000, make string each char in ss denote 1~16 in sequence.
	  {
		if(i<8)
		{
		  ss[i] = s_l[7-i];
		}
		else
		  ss[i] = s_h[15-i];		
	  }
	
	  int sum_activated = 0;
	  int sum_id = 0;
	  for(i=0;i<16;i++)
	  {
		if(ss[i]=='1')
		{
		  sum_id += i+1;
		  sum_activated += 1;
		}		
	  }

	  double lat_dev_mgs = 0;
	  lat_dev_mgs = double(sum_id)/double(sum_activated)-8.5;
	  
	  magnetic_writer_->Write(lat_dev_mgs);

      // magnetic.set_id(static_cast<int>(buffer[10]));

      magnetic_writer_->Write(magnetic);
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
  apollo::cyber::WaitForShutdown();
  return 0;
}