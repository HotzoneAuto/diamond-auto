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

#include "modules/canbus/vehicle/diamond/protocol/id_0x18fe0010.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x18fe0010::Id0x18fe0010() {}
const int32_t Id0x18fe0010::ID = 0x18FE0010;

void Id0x18fe0010::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x18fe0010()->set_fmt0(
      fmt0(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x18fe0010()->set_fmt1(
      fmt1(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x18fe0010()->set_fmt2(
      fmt2(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x18fe0010()->set_fmt3(
      fmt3(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x18fe0010()->set_fmt4(
      fmt4(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x18fe0010()->set_fmt5(
      fmt5(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x18fe0010()->set_fmt6(
      fmt6(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x18fe0010()->set_parking_mode(
      parking_mode(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'fmt0',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
int Id0x18fe0010::fmt0(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'fmt1',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
int Id0x18fe0010::fmt1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'fmt2',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
int Id0x18fe0010::fmt2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'fmt3',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
int Id0x18fe0010::fmt3(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'fmt4',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
int Id0x18fe0010::fmt4(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name': 'fmt5',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
int Id0x18fe0010::fmt5(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'fmt6',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
int Id0x18fe0010::fmt6(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'parking_mode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x18fe0010::parking_mode(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
