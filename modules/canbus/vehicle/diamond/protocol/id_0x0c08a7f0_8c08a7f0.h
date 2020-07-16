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
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'fMotTq', 'offset': -32000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-32000|33535]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'NM'}
  int fmottq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'fMotSpd', 'offset': 0.0, 'precision': 0.5, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|32767.5]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'RPM'}
  double fmotspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'fMotCur', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|5553.5]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': 'A'}
  double fmotcur(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'byMCUSts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 48, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bymcusts1(const std::uint8_t* bytes, const int32_t length) const;

  // // config detail: {'name': 'byMCUSts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 49, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  // bool bymcusts(const std::uint8_t* bytes, const int32_t length) const;

  // // config detail: {'name': 'byMCUSts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 50, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  // bool bymcusts(const std::uint8_t* bytes, const int32_t length) const;

  // // config detail: {'name': 'byMCUSts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 51, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  // bool bymcusts(const std::uint8_t* bytes, const int32_t length) const;

  // // config detail: {'name': 'byMCUSts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 52, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  // bool bymcusts(const std::uint8_t* bytes, const int32_t length) const;

  // // config detail: {'name': 'byMCUSts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 53, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  // bool bymcusts(const std::uint8_t* bytes, const int32_t length) const;

  // // config detail: {'name': 'byMCUSts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 54, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  // bool bymcusts(const std::uint8_t* bytes, const int32_t length) const;

  // // config detail: {'name': 'byMCUSts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 55, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  // bool bymcusts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'byMCU1MsgLife1', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int bymcu1msglife1(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


