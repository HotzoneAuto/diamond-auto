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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c09a7f0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x0c09a7f0::Id0x0c09a7f0() {}
const int32_t Id0x0c09a7f0::ID = 0x0C09A7F0;

void Id0x0c09a7f0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  auto diamond = chassis->mutable_diamond();
  diamond->mutable_id_0x0c09a7f0()->set_fmotvolt(fmotvolt(bytes, length));
  diamond->mutable_id_0x0c09a7f0()->set_fmottemp(fmottemp(bytes, length));
  diamond->mutable_id_0x0c09a7f0()->set_fmcutemp(fmcutemp(bytes, length));
  diamond->mutable_id_0x0c09a7f0()->set_fmotrectcur(fmotrectcur(bytes, length));
  diamond->mutable_id_0x0c09a7f0()->set_bymotspddir(bymotspddir(bytes, length));
  diamond->mutable_id_0x0c09a7f0()->set_bymoterrcode(
      bymoterrcode(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
// 'fmotvolt', 'offset': -1000.0, 'order': 'intel', 'physical_range':
// '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Id0x0c09a7f0::fmotvolt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -1000.000000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
// 'fmottemp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|215]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x0c09a7f0::fmottemp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
// 'fmcutemp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|215]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x0c09a7f0::fmcutemp(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name':
// 'fmotrectcur', 'offset': -1000.0, 'order': 'intel', 'physical_range':
// '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Id0x0c09a7f0::fmotrectcur(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -1000.000000;
  return ret;
}

// config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name':
// 'bymotspddir', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x0c09a7f0::bymotspddir(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'bymoterrcode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x0c09a7f0::bymoterrcode(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
