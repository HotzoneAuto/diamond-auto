#include "modules/sensors/wr_ls/wr_ls_sensor_frame.h"
#include "modules/sensors/wr_ls/wr_ls_constants.h"
namespace apollo {
namespace sensors {
namespace wr_ls {
CWrLsSensFrame::CWrLsSensFrame() {
  mSensDataLength = 0;
  m_pSensData = NULL;
}

CWrLsSensFrame::~CWrLsSensFrame() {
  if (m_pSensData != NULL) delete m_pSensData;
}

uint8_t CWrLsSensFrame::GetFrameHeader() { return m_pSensData->header; }

uint8_t CWrLsSensFrame::GetCommandId() { return m_pSensData->cmd_id; }

uint16_t CWrLsSensFrame::GetRangeStart() { return m_pSensData->range_start; }

uint16_t CWrLsSensFrame::GetRangeEnd() { return m_pSensData->range_end; }

int CWrLsSensFrame::GetSensDataCount() {
  return m_pSensData->range_end - m_pSensData->range_start + 1;
}

uint16_t CWrLsSensFrame::GetSensDataOfIndex(int index) {
  if (index < 0 ||
      index > (m_pSensData->range_end - m_pSensData->range_start)) {
    AERROR << "Fail to get of index " << index;
    return 0;
  }

  return m_pSensData->sens_data[index];
}

bool CWrLsSensFrame::CheckFrame(char *buff, int length, uint8_t value) {
  int i = 0;
  uint8_t result = 0;
  bool checkframe;

  /* Get configure form launch script */
  ros::param::get("~checkframe", checkframe);
  if (checkframe == false) {
    /* Disable check frame function, default check true*/
    return true;
  }

  if (buff == NULL || length <= 0) {
    printf("CheckFrame: parameter failed\n");
    return false;
  }

  for (i = 0; i < length; i++) {
    result += (*buff++);
  }

  if (result == value) {
    return true;
  }

  printf("CheckFrame: check failed, length = %d, result = 0x%X, value = 0x%X\n",
         length, result, value);

  return false;
}

bool CWrLsSensFrame::InitFromSensBuff(char *buff, int length) {
  if (buff == NULL) {
    AERROR << "Invalide input buffer!";
    return false;
  }

  char *pData = new char[length];
  if (pData == NULL) {
    AERROR << "Insufficiant memory!";
    return NULL;
  }

  memcpy(pData, buff, length);
  m_pSensData = new (pData) CWrLsSensFrame::SensData;
  mSensDataLength = length;

  /*System is using LOW END ENCODING, swtich the words*/
  m_pSensData->range_start = SWITCH_UINT16(m_pSensData->range_start);
  m_pSensData->range_end = SWITCH_UINT16(m_pSensData->range_end);

  /*Switch sensor data*/
  int dataCount = this->GetSensDataCount();

  if (true != CheckFrame((char *)m_pSensData->sens_data, dataCount * 2,
                         m_pSensData->check_value)) {
    AERROR << "CheckFrame failed";
    return false;
  }

  int index = 0;
  while (index < dataCount) {
    m_pSensData->sens_data[index] =
        SWITCH_UINT16(m_pSensData->sens_data[index]);
    index++;
  }

  return true;
}

void CWrLsSensFrame::DumpFrameHeader() {
  if (m_pSensData == NULL || mSensDataLength == 0) {
    return;
  }

  // TODO
  ADEBUG << "Frame Header: 0x%02X" << this->GetFrameHeader();
  ADEBUG << "Command   ID: 0x%02X" << this->GetCommandId();
  ADEBUG << "Angle  START: 0x%04X" << this->GetRangeStart();
  ADEBUG << "Angle    END: 0x%04X" << this->GetRangeEnd();
}

void CWrLsSensFrame::DumpFrameData() {
  if (m_pSensData == NULL || mSensDataLength == 0) {
    return;
  }

  int dataCount = this->GetSensDataCount();
  ADEBUG << "Data   Count: " << dataCount;

  int idx = 1;
  while (idx <= dataCount) {
    printf("%u ", static_cast<unsigned int>(this->GetSensDataOfIndex(idx - 1)));

    idx++;
    if (idx % 48 == 0) printf("\n");
  }
  printf("\n");
}

}  // namespace wr_ls
}  // namespace sensors
}  // namespace apollo
