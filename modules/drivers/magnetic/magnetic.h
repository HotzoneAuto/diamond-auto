#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/cyber.h"

#include "modules/common/util/uart.h"

namespace apollo {
namespace drivers {
namespace magnetic {

float getLatdev(int dec) {
  long long int bin = 0;
  int temp = dec;
  long long j = 1;
  while (temp) {
    bin = bin + j * (temp % 2);
    temp = temp / 2;
    j = j * 10;
  }
  std::string s = std::to_string(bin);
  while (s.size() < 16) {
    s = '0' + s;
  }
  int sum_activated = 0;
  int sum_id = 0;
  for (int i = 0; i < 16; i++) {
    if (s[i] == '1') {
      sum_id += 16 - i;
      sum_activated += 1;
    }
  }
  auto lat_dev_mgs =
      static_cast<float>(sum_id) / static_cast<float>(sum_activated) - 8.5;
  return lat_dev_mgs;
}

class Magnetic {
 public:
  void AsyncSend();
};

}  // namespace magnetic
}  // namespace drivers
}  // namespace apollo
