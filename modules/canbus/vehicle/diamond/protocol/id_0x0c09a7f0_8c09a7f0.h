/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace diamond {

class Id0x0c09a7f08c09a7f0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x0c09a7f08c09a7f0();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'fMotVolt', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  Id0x0c09a7f08c09a7f0* set_fmotvolt(double fmotvolt);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'fMotTemp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|215]', 'physical_unit': 'TMP', 'precision': 1.0, 'type': 'int'}
  Id0x0c09a7f08c09a7f0* set_fmottemp(int fmottemp);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'fMCUTemp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|215]', 'physical_unit': 'TMP', 'precision': 1.0, 'type': 'int'}
  Id0x0c09a7f08c09a7f0* set_fmcutemp(int fmcutemp);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'fMotRectCur', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  Id0x0c09a7f08c09a7f0* set_fmotrectcur(double fmotrectcur);

  // config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name': 'byMotSpdDir', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c09a7f08c09a7f0* set_bymotspddir(bool bymotspddir);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'byMotErrCode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c09a7f08c09a7f0* set_bymoterrcode(int bymoterrcode);

 private:

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'fMotVolt', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  void set_p_fmotvolt(uint8_t* data, double fmotvolt);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'fMotTemp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|215]', 'physical_unit': 'TMP', 'precision': 1.0, 'type': 'int'}
  void set_p_fmottemp(uint8_t* data, int fmottemp);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'fMCUTemp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|215]', 'physical_unit': 'TMP', 'precision': 1.0, 'type': 'int'}
  void set_p_fmcutemp(uint8_t* data, int fmcutemp);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'fMotRectCur', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  void set_p_fmotrectcur(uint8_t* data, double fmotrectcur);

  // config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name': 'byMotSpdDir', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_bymotspddir(uint8_t* data, bool bymotspddir);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'byMotErrCode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_bymoterrcode(uint8_t* data, int bymoterrcode);

 private:
  double fmotvolt_;
  int fmottemp_;
  int fmcutemp_;
  double fmotrectcur_;
  bool bymotspddir_;
  int bymoterrcode_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


