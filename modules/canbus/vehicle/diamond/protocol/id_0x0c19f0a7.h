/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace diamond {

class Id0x0c19f0a7 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x0c19f0a7();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMot1TargetTq', 'offset': -32000.0, 'order': 'intel', 'physical_range':
  // '[-32000|33535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
  Id0x0c19f0a7* set_fmot1targettq(int fmot1targettq);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMot1LmtVolt', 'offset': -1000.0, 'order': 'intel', 'physical_range':
  // '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  Id0x0c19f0a7* set_fmot1lmtvolt(double fmot1lmtvolt);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMot1LmtCur', 'offset': -1000.0, 'order': 'intel', 'physical_range':
  // '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  Id0x0c19f0a7* set_fmot1lmtcur(double fmot1lmtcur);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'byMot1WorkMode', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c19f0a7* set_bymot1workmode(int bymot1workmode);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'byLife', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c19f0a7* set_bylife(int bylife);

 private:
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMot1TargetTq', 'offset': -32000.0, 'order': 'intel', 'physical_range':
  // '[-32000|33535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
  void set_p_fmot1targettq(uint8_t* data, int fmot1targettq);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMot1LmtVolt', 'offset': -1000.0, 'order': 'intel', 'physical_range':
  // '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  void set_p_fmot1lmtvolt(uint8_t* data, double fmot1lmtvolt);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMot1LmtCur', 'offset': -1000.0, 'order': 'intel', 'physical_range':
  // '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  void set_p_fmot1lmtcur(uint8_t* data, double fmot1lmtcur);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'byMot1WorkMode', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_bymot1workmode(uint8_t* data, int bymot1workmode);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'byLife', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_bylife(uint8_t* data, int bylife);

 private:
  int fmot1targettq_;
  double fmot1lmtvolt_;
  double fmot1lmtcur_;
  int bymot1workmode_;
  int bylife_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
