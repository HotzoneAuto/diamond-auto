#pragma once

#include <stdio.h>
#include <stdlib.h>

// #include <ros/ros.h>
#include "cyber/common/log.h"
#include "cyber/cyber.h"

namespace apollo {
namespace sensors {
namespace wr_ls {

class CWrLsSensFrame {
  struct SensData {
    uint8_t header;
    uint8_t cmd_id;
    uint16_t range_start;
    uint16_t range_end;
    uint8_t check_value;
    uint16_t sens_data[0];
  } __attribute__((packed));

 public:
  CWrLsSensFrame();
  ~CWrLsSensFrame();

  bool InitFromSensBuff(char *buff, int length);

  /*Get Frame Header*/
  uint8_t GetFrameHeader();

  /*Get command Id*/
  uint8_t GetCommandId();

  /*Get Range Start and Range End*/
  uint16_t GetRangeStart();
  uint16_t GetRangeEnd();

  /*Get sensor data count*/
  int GetSensDataCount();

  /*Get sensor data of index*/
  uint16_t GetSensDataOfIndex(int index);

  /*For debug only: Dump frame header.*/
  void DumpFrameHeader();

  /*For debug only: Dump frame data.*/
  void DumpFrameData();

 private:
  SensData *m_pSensData;
  int mSensDataLength;

  bool CheckFrame(char *buff, int length, uint8_t value);
};
}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo

