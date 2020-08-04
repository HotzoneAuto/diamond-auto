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

class Id0x0c08a7f0 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x0c08a7f0();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMotTq', 'offset': -32000.0, 'order': 'intel', 'physical_range':
  // '[-32000|33535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
  int fmottq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMotSpd', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|32767.5]', 'physical_unit': 'RPM', 'precision': 0.5, 'type': 'double'}
  double fmotspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMotCur', 'offset': -1000.0, 'order': 'intel', 'physical_range':
  // '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double fmotcur(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bymcusts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'byMCU1MsgLife1', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bymcu1msglife1(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
