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

#include "modules/canbus/vehicle/diamond/protocol/id_0x03.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x03::ID = 0x03;

Id0x03::Id0x03() { Reset(); }

uint32_t Id0x03::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x03::UpdateData(uint8_t* data) { set_p_front_mgs(data, front_mgs_); }

void Id0x03::Reset() {
  // TODO(All) :  you should check this manually
  front_mgs_ = 0;
}

Id0x03* Id0x03::set_front_mgs(int front_mgs) {
  front_mgs_ = front_mgs;
  return this;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 16, 'name':
// 'Front_MGS', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x03::set_p_front_mgs(uint8_t* data, int front_mgs) {
  front_mgs = ProtocolData::BoundedValue(0, 0, front_mgs);
  int x = front_mgs;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 7);
  to_set1.set_value(t, 0, 8);
}

void Id0x03::Parse(const std::uint8_t* bytes, int32_t length,
                   ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x03()->set_front_mgs(
      front_mgs(bytes, length));
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 16, 'name':
// 'front_mgs', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x03::front_mgs(const std::uint8_t* bytes, int32_t length) const {
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
