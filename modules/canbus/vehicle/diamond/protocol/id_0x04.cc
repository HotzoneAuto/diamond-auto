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

#include "modules/canbus/vehicle/diamond/protocol/id_0x04.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x04::Id0x04() { Reset(); }
const int32_t Id0x04::ID = 0x04;

uint32_t Id0x04::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x04::UpdateData(uint8_t* data) { set_p_rear_mgs(data, rear_mgs_); }

void Id0x04::Reset() {
  // TODO(All) :  you should check this manually
  rear_mgs_ = 0;
}

Id0x04* Id0x04::set_rear_mgs(int rear_mgs) {
  rear_mgs_ = rear_mgs;
  return this;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 16, 'name':
// 'Rear_MGS', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x04::set_p_rear_mgs(uint8_t* data, int rear_mgs) {
  rear_mgs = ProtocolData::BoundedValue(0, 0, rear_mgs);
  int x = rear_mgs;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 7);
  to_set1.set_value(t, 0, 8);
}

void Id0x04::Parse(const std::uint8_t* bytes, int32_t length,
                   ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x04()->set_rear_mgs(
      rear_mgs(bytes, length));
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 16, 'name':
// 'rear_mgs', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x04::rear_mgs(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
