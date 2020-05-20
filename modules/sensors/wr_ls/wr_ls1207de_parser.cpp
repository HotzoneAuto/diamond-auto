#include "modules/sensors/wr_ls/wr_ls1207de_parser.h"

// #include <ros/ros.h>
#include <stdio.h>
#include "modules/sensors/proto/laser_scan.pb.h"
#include "modules/sensors/wr_ls/wr_ls_sensor_frame.h"

namespace apollo {
namespace sensors {
namespace wr_ls {

using apollo::cyber::Time;

CWrLs1207DEParser::CWrLs1207DEParser()
    : CParserBase(), fRangeMin(0.05), fRangeMax(10.0), fTimeIncrement(-1.0) {
  // Do Nothing...
}

int CWrLs1207DEParser::Parse(char *data, size_t data_length, WrLsConfig &config,
                             LaserScan &msg) {
  CWrLsSensFrame *pSensFrame = new CWrLsSensFrame();
  if (!pSensFrame->InitFromSensBuff(data, data_length)) {
    AINFO << "Invalid frame data!";
    return ExitSuccess;
  }

  int dataCount = pSensFrame->GetSensDataCount();

  /*Fill sensor message struct*/
  msg.mutable_header()->set_frame_id(config.frame_id());
  ADEBUG << "Publishing with frame id: " << config.frame_id();

  /*1: Scan time: The time for every frame.*/
  Time start_time = Time::Now();
  unsigned short scanning_freq =
      1000 / 43 * 100; /*For dev borad, the device will send data every 50ms*/
  msg.set_scan_time(1.0 / (scanning_freq / 100.0));
  ADEBUG << "scanning freq: " << scanning_freq
         << " ,scan_time: " << msg.scan_time();

  /*2: Time increment: Time interval for between each data.*/
  /*Time increment has been overriden*/
  fTimeIncrement = 0.000040;
  msg.set_time_increment(fTimeIncrement);
  ADEBUG << "time_increment: " << msg.time_increment();

  /*3: Angle Min: Starting angle of current scanning data.*/
  int starting_angle = 0xFFF92230; /*This value is from Sick Tim*/
  msg.set_angle_min((starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2);
  ADEBUG << "starting_angle: " << starting_angle
         << " angle_min: " << msg.angle_min();

  /*4: Angle step width: anguler between each scanning data.*/
  unsigned short angular_step_width =
      0xD05; /*3333: This value is from Sick Tim*/
  msg.set_angle_increment((angular_step_width / 10000.0) / 180.0 * M_PI);

  /*5: Angle Max: Ending angle of current scanning data.*/
  msg.set_angle_max(msg.angle_min() + (dataCount - 1) * msg.angle_increment());

  /* calculate statring data index and adjust angle_min to min_ang config param
   */
  int index_min = 0;
  while (msg.angle_min() + msg.angle_increment() < config.min_ang()) {
    msg.set_angle_min(msg.angle_min() + msg.angle_increment());
    index_min++;
  }
  ADEBUG << "index_min: " << index_min << " angle_min: " << msg.angle_min();

  /* calculate ending data index and adjust angle_max to max_ang config param */
  int index_max = dataCount - 1;
  while (msg.angle_max() - msg.angle_increment() > config.max_ang()) {
    // msg.angle_max -= msg.angle_increment();
    msg.set_angle_max(msg.angle_max() - msg.angle_increment());
    index_max--;
  }
  ADEBUG << "index_max: " << index_max << " ,angle_max: " << msg.angle_max();

  /*5: Fill data range*/
  // TODO(fengzongbao)
  // msg.ranges.resize(index_max - index_min + 1);
  ADEBUG << "Fill sensor data. index_min = " << index_min
         << "index_max = " << index_max;
  // for (int j = index_min; j <= index_max; ++j) {
  //   if (config.debug_mode()) {
  //     if ((j - index_min + 1) % 48 == 0) {
  //       printf("\n");
  //     }
  //   }

  //   unsigned short range = pSensFrame->GetSensDataOfIndex(j);
  //   /*unsigned short range = 3000; */ /*For testing....*/
  //   // TODO(FENGZONGBAO)
  //   // msg.ranges[j - index_min] = range / 100.0;

  //   if (config.debug_mode()) {
  //     printf("%.2f ", msg.ranges[j - index_min]);
  //   }
  // }
  if (config.debug_mode()) printf("\n");

  /*Override range*/
  msg.set_range_min(fRangeMin);
  msg.set_range_max(fRangeMax);

  /*6: Setting starting time*/
  /* - last scan point = now ==> first scan point = now - data count * time
   * increment*/
  // msg.header.stamp = start_time - ros::Duration().fromSec(dataCount * msg.time_increment);
  msg.mutable_header()->set_timestamp_sec(
      start_time.ToSecond() - apollo::cyber::Duration(dataCount * msg.time_increment()).ToSecond());
  /* - shit forward to time of first published scan point*/
  // msg.header.stamp += ros::Duration().fromSec((double)index_min * msg.time_increment);
  msg.mutable_header()->set_timestamp_sec(
      msg.header().timestamp_sec() +
      apollo::cyber::Duration((double)index_min * msg.time_increment()).ToSecond());

  /* - add time offset (to account for USB latency etc.)*/
  // msg.header.timestamp_sec += apollo::cyber::Duration(config.time_offset());
  msg.mutable_header()->set_timestamp_sec(msg.header().timestamp_sec() + apollo::cyber::Duration(config.time_offset()).ToSecond());

  /*Consistency Check*/
  float expected_time_increment =
      msg.scan_time() * msg.angle_increment() / (2.0 * M_PI);
  if (fabs(expected_time_increment - msg.time_increment()) > 0.00001) {
    AINFO_EVERY(60) << "The time_increment, scan_time and angle_increment "
                       "values reported by "
                       "the scanner are inconsistent! "
                       "Expected time_increment: "
                    << expected_time_increment
                    << ", reported time_increment: " << msg.time_increment()
                    << "Perhaps you should set the parameter time_increment to "
                       "the expected "
                       "value. This message will print every 60 seconds.";
  }

  return ExitSuccess;
}

void CWrLs1207DEParser::SetRangeMin(float minRange) { fRangeMin = minRange; }

void CWrLs1207DEParser::SetRangeMax(float maxRange) { fRangeMax = maxRange; }

void CWrLs1207DEParser::SetTimeIncrement(float time) { fTimeIncrement = time; }

CWrLs1207DEParser::~CWrLs1207DEParser() {
  // Do Nothing...
}
}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo
