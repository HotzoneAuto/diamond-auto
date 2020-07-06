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

class Id0x0c08a7f08c08a7f0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x0c08a7f08c08a7f0();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'fMotTq', 'offset': -32000.0, 'order': 'intel', 'physical_range': '[-32000|33535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
  Id0x0c08a7f08c08a7f0* set_fmottq(int fmottq);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'fMotSpd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|32767.5]', 'physical_unit': 'RPM', 'precision': 0.5, 'type': 'double'}
  Id0x0c08a7f08c08a7f0* set_fmotspd(double fmotspd);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'fMotCur', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  Id0x0c08a7f08c08a7f0* set_fmotcur(double fmotcur);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c08a7f08c08a7f0* set_bymcusts(bool bymcusts);

  // config detail: {'bit': 49, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c08a7f08c08a7f0* set_bymcusts(bool bymcusts);

  // config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c08a7f08c08a7f0* set_bymcusts(bool bymcusts);

  // config detail: {'bit': 51, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c08a7f08c08a7f0* set_bymcusts(bool bymcusts);

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c08a7f08c08a7f0* set_bymcusts(bool bymcusts);

  // config detail: {'bit': 53, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c08a7f08c08a7f0* set_bymcusts(bool bymcusts);

  // config detail: {'bit': 54, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c08a7f08c08a7f0* set_bymcusts(bool bymcusts);

  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c08a7f08c08a7f0* set_bymcusts(bool bymcusts);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'byMCU1MsgLife1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c08a7f08c08a7f0* set_bymcu1msglife1(int bymcu1msglife1);

 private:

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'fMotTq', 'offset': -32000.0, 'order': 'intel', 'physical_range': '[-32000|33535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
  void set_p_fmottq(uint8_t* data, int fmottq);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'fMotSpd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|32767.5]', 'physical_unit': 'RPM', 'precision': 0.5, 'type': 'double'}
  void set_p_fmotspd(uint8_t* data, double fmotspd);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'fMotCur', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  void set_p_fmotcur(uint8_t* data, double fmotcur);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_bymcusts(uint8_t* data, bool bymcusts);

  // config detail: {'bit': 49, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_bymcusts(uint8_t* data, bool bymcusts);

  // config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_bymcusts(uint8_t* data, bool bymcusts);

  // config detail: {'bit': 51, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_bymcusts(uint8_t* data, bool bymcusts);

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_bymcusts(uint8_t* data, bool bymcusts);

  // config detail: {'bit': 53, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_bymcusts(uint8_t* data, bool bymcusts);

  // config detail: {'bit': 54, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_bymcusts(uint8_t* data, bool bymcusts);

  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_bymcusts(uint8_t* data, bool bymcusts);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'byMCU1MsgLife1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_bymcu1msglife1(uint8_t* data, int bymcu1msglife1);

 private:
  int fmottq_;
  double fmotspd_;
  double fmotcur_;
  bool bymcusts_;
  bool bymcusts_;
  bool bymcusts_;
  bool bymcusts_;
  bool bymcusts_;
  bool bymcusts_;
  bool bymcusts_;
  bool bymcusts_;
  int bymcu1msglife1_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


