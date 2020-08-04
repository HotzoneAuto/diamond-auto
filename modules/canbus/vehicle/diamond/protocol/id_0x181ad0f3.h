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

class Id0x181ad0f3 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x181ad0f3();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'fPosInsRValue', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'k', 'precision': 1.0, 'type': 'int'}
  int fposinsrvalue(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'fNegInsRValue', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'k', 'precision': 1.0, 'type': 'int'}
  int fneginsrvalue(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo

