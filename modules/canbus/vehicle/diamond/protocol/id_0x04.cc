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

Id0x04::Id0x04() {}
const int32_t Id0x04::ID = 0x04;

void Id0x04::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x04()->set_rear_mgs(rear_mgs(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x04()->set_format0_re(format0_re(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x04()->set_format1_re(format1_re(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x04()->set_format2_re(format2_re(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x04()->set_format3_re(format3_re(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x04()->set_format4_re(format4_re(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x04()->set_format5_re(format5_re(bytes, length));
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 16, 'name': 'rear_mgs', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65536]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
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

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'format0_re', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x04::format0_re(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'format1_re', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x04::format1_re(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'format2_re', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x04::format2_re(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'format3_re', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x04::format3_re(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'format4_re', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x04::format4_re(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name': 'format5_re', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|10]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x04::format5_re(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
