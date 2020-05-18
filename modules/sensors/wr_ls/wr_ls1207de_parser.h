#pragma once

#include "modules/sensors/wr_ls/parser_base.h"
#include "modules/sensors/proto/laser_config.pb.h"
#include "modules/sensors/proto/laser_scan.pb.h"

namespace apollo {
namespace sensors {
namespace wr_ls {

using apollo::sensors::WrLsConfig;
using apollo::sensors::LaserScan;

class CWrLs1207DEParser : public CParserBase {
 public:
  CWrLs1207DEParser();
  virtual ~CWrLs1207DEParser();

  virtual int Parse(char *data, size_t data_length, WrLsConfig &config,
                    LaserScan &msg);

  void SetRangeMin(float minRange);
  void SetRangeMax(float maxRange);
  void SetTimeIncrement(float time);

 private:
  float fRangeMin;
  float fRangeMax;
  float fTimeIncrement;
};
}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo

