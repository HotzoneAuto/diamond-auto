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

class Id0x0c09a7f08c09a7f0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x0c09a7f08c09a7f0();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'fMotVolt', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|5553.5]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': 'V'}
  double fmotvolt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'fMotTemp', 'offset': -40.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-40|215]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': 'TMP'}
  int fmottemp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'fMCUTemp', 'offset': -40.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[-40|215]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': 'TMP'}
  int fmcutemp(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'fMotRectCur', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|5553.5]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': 'A'}
  double fmotrectcur(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'byMotSpdDir', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 50, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool bymotspddir(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'byMotErrCode', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int bymoterrcode(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


