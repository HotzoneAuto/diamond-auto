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

class Id0x03 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x03();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 16, 'name': 'Front_MGS', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65536]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int front_mgs(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'Format0_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int format0_fr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'Format1_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int format1_fr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'Format2_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int format2_fr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'Format3_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int format3_fr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'Format4_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int format4_fr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name': 'Format5_fr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int format5_fr(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


