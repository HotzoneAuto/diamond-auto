#include "modules/drivers/magnetic/magnetic.h"

#include <stdlib.h> /* system, NULL, EXIT_FAILURE */

namespace apollo {
namespace drivers {
namespace magnetic {

void Magnetic::AsyncSend() {
  std::vector<std::string> cmds = {"cansend can0 003#0102030405010000",
                                   "cansend can0 004#0102030405010000"};
  while (!apollo::cyber::IsShutdown()) {
    for (auto& cmd : cmds) {
      const int ret = std::system(cmd.c_str());
      if (ret == 0) {
        AINFO << "Magnetic can message send SUCCESS: " << cmd;
      } else {
        AERROR << "Magnetic can message send FAILED(" << ret << "): " << cmd;
      }
    }
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));
  }
}
}  // namespace magnetic
}  // namespace drivers
}  // namespace apollo
