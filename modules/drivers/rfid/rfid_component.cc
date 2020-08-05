
#include "modules/drivers/rfid/rfid_component.h"

#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace drivers {
namespace rfid {

int stringToHex(char *str, uint8_t *out) {
  int outLength = 0;
  int cnt = 0;
  uint8_t high = 0, low = 0;
  char current = 0;
  uint8_t value = 0;
  uint8_t isHighValid = 0;
  while ((current = str[cnt])) {
    ++cnt;
    if (current >= '0' && current <= '9') {
      value = static_cast<uint8_t>(current - '0');
    } else if (current >= 'a' && current <= 'f') {
      value = static_cast<uint8_t>(current - 'a' + 10);
    } else if (current >= 'A' && current <= 'F') {
      value = static_cast<uint8_t>(current - 'A' + 10);
    } else {
      continue;
    }

    if (!isHighValid) {
      high = value;

      isHighValid = 1;
    } else {
      low = value;

      out[outLength] = static_cast<uint8_t>(high << 4) | low;
      ++outLength;
      isHighValid = 0;
    }
  }

  return outLength;
}

bool RfidComponent::Init() {
  char str = 0x03;
  uint8_t out;
  stringToHex(&str, &out);
  AINFO << out;
  // Uart device set option
  device_.SetOpt(9600, 8, 'N', 1);

  // Publish rfid station data
  rfid_writer_ = node_->CreateWriter<RFID>(FLAGS_rfid_topic);

  // Async read
  async_action_ = cyber::Async(&RfidComponent::Action, this);
  return true;
}

// TODO()CHECK
bool RfidComponent::Check() {
  // sum check for rfid result
  int check_value = 0;// buffer[11];

  // char new_hex;
  // auto transfered_size = stringToHex(&buffer + 1, &new_hex);
  // AINFO << "transfered size : " << transfered_size
  //       << " retrun transfered: " << new_hex;

  int value;
  // for (int i = 0; i < 11; i + 2) {
    // value | = new_hex[i];
  // }

  return value == check_value;
}

void RfidComponent::Action() {
  int count = 0;
  static char buffer[13];
  static char buf;
  while (!apollo::cyber::IsShutdown()) {
    // count = 1;
    std::memset(buffer, 0, 13);
    while (1) {
      int ret = device_.Read(&buf, 1);
      AINFO << "RFID Device return: " << ret;
      if (ret == 1) {
        AINFO << "RFID Device buf: " << buf;
        // 0x02 Head
        // 0x03 End
        if (buf == 0x02) {
          count = 0;
          break;
        }
        buffer[count] = buf;
        count++;
      }
      AINFO << "count: " << count;
      // 0x03 end
      if (buf == 0x03 && count == 12) {
        RFID rfid;
        rfid.set_id(static_cast<int>(buffer[10]));
        rfid.set_description("CARD STATION");

        rfid_writer_->Write(rfid);
      }
    }
  }
}

RfidComponent::~RfidComponent() { AINFO << "RfidComponent::~RfidComponent()"; }

}  // namespace rfid
}  // namespace drivers
}  // namespace apollo
