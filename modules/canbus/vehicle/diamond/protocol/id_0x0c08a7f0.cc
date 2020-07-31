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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c08a7f0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x0c08a7f0::Id0x0c08a7f0() {}
const int32_t Id0x0c08a7f0::ID = 0x0C08A7F0;

void Id0x0c08a7f0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0()->set_fmottq(
      fmottq(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0()->set_fmotspd(
      fmotspd(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0()->set_fmotcur(
      fmotcur(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0()->set_bymcusts(
      bymcusts(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0()->set_bymcu1msglife1(
      bymcu1msglife1(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
// 'fmottq', 'offset': -32000.0, 'order': 'intel', 'physical_range':
// '[-32000|33535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
int Id0x0c08a7f0::fmottq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -32000.000000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name':
// 'fmotspd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|32767.5]',
// 'physical_unit': 'RPM', 'precision': 0.5, 'type': 'double'}
double Id0x0c08a7f0::fmotspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.500000;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name':
// 'fmotcur', 'offset': -1000.0, 'order': 'intel', 'physical_range':
// '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Id0x0c08a7f0::fmotcur(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -1000.000000;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
// 'bymcusts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x0c08a7f0::bymcusts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'bymcu1msglife1', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x0c08a7f0::bymcu1msglife1(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
