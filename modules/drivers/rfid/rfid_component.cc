
#include "modules/drivers/rfid/rfid_component.h"

namespace apollo {
namespace drivers {
namespace rfid {

bool RfidComponent::Init() {
  device_.SetOpt(9600, 8, 'N', 1);

  rfid_writer_ = 
        node_->CreateWriter<RFID>("/diamond/sensor/rfid");

  async_action_ = cyber::Async(&RfidComponent::Action, this);
  return true;
}

void RfidComponent::Action(){
  int count = 1;
  static char buffer[20];
  static char buf;
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

        RFID rfid;
        rfid.set_id(static_cast<int>(buffer[10]));

        rfid_writer_->Write(rfid);
      }
    }
    
  }
}

RfidComponent::~RfidComponent(){
  AINFO << "RfidComponent::~RfidComponent()";
}

}  // namespace rfid
}  // namespace drivers
}  // namespace apollo
