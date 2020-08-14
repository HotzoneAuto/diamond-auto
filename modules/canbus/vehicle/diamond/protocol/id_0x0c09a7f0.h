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

class Id0x0c09a7f0 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x0c09a7f0();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMotVolt', 'offset': -1000.0, 'order': 'intel', 'physical_range':
  // '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double fmotvolt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
  // 'fMotTemp', 'offset': -40.0, 'order': 'intel', 'physical_range':
  // '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int fmottemp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
  // 'fMCUTemp', 'offset': -40.0, 'order': 'intel', 'physical_range':
  // '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int fmcutemp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name':
  // 'fMotRectCur', 'offset': -1000.0, 'order': 'intel', 'physical_range':
  // '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double fmotrectcur(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name':
  // 'byMotSpdDir', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bymotspddir(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'byMotErrCode', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bymoterrcode(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
