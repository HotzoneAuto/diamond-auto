#include <string.h>
#include <cmath>

#include "cyber/time/rate.h"
#include "cyber/time/time.h"

#include "modules/drivers/parking/parking_brake.h"

namespace apollo {
namespace drivers {
namespace parking {

using apollo::cyber::Rate;

bool ParkingComponet::Init() {
  if (!GetProtoConfig(&device_conf_)) {
    AERROR << "Unable to load parking_brake conf file: " << ConfigFilePath();
    return false;
  }
  device_ = std::make_unique<Uart>(device_conf_.device_id().c_str());
  // Uart device set option
  device_->SetOpt(38400, 8, 'N', 1);
  // Publish rfid station data
  parking_writer_ = node_->CreateWriter<PARKING>(device_conf_.output_channel());

  // Async read
  async_action_ = cyber::Async(&ParkingComponet::Proc, this);
  return true;
}
void ParkingComponet::Proc() {
  AINFO << "Parking Proc";
  static char buffer[6];
  static char buf;
  int count = 0;
  double air_pump_pressure = 0.0;
  //    uint8_t station_id;
  unsigned char table[8] = {0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xC5, 0xCB};
  Rate rate(100.0);
  while (!apollo::cyber::IsShutdown()) {
    count = 0;
    std::memset(buffer, 0, 6);
    //while(1){
    int results = device_->Write(table, 8);
    AINFO << "results==" << results;
    for (count = 0; count < 7; count++) {
      int ret = device_->Read(&buf, 1);
      ADEBUG << "READ RETURN :" << ret;
      if (ret == 1) {
        buffer[count] = buf;
      } else {
        std::memset(buffer, 0, 6);
        break;
      }
      if (count == 6) {
        AINFO << "buffer[0]=" << buffer[0];
        air_pump_pressure = (static_cast<double>(buffer[3]) * 256 +
                             static_cast<double>(buffer[4])) /
                            100.0;
        AINFO << "air_pump_pressure" << air_pump_pressure;
      }
    }
    auto header = parking.mutable_header();
    header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
    header->set_frame_id("parking");
    parking.set_barometric_pressure(air_pump_pressure);
    parking_writer_->Write(parking);
    rate.Sleep();
  }
}
}  // namespace parking
}  // namespace drivers
}  // namespace apollo
