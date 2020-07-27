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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c08a7f0_8c08a7f0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x0c08a7f08c08a7f0::Id0x0c08a7f08c08a7f0() {}
const int32_t Id0x0c08a7f08c08a7f0::ID = 0x0c08a7f0;

void Id0x0c08a7f08c08a7f0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_fmottq(fmottq(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_fmotspd(fmotspd(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_fmotcur(fmotcur(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_bymcusts1(bymcusts1(bytes, length));
  // chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_bymcusts(bymcusts(bytes, length));
  // chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_bymcusts(bymcusts(bytes, length));
  // chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_bymcusts(bymcusts(bytes, length));
  // chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_bymcusts(bymcusts(bytes, length));
  // chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_bymcusts(bymcusts(bytes, length));
  // chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_bymcusts(bymcusts(bytes, length));
  // chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_bymcusts(bymcusts(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c08a7f0_8c08a7f0()->set_bymcu1msglife1(bymcu1msglife1(bytes, length));
}

// config detail: {'name': 'fmottq', 'offset': -32000.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[-32000|33535]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'NM'}
int Id0x0c08a7f08c08a7f0::fmottq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -32000.000000;
  return ret;
}

// config detail: {'name': 'fmotspd', 'offset': 0.0, 'precision': 0.5, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|32767.5]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'RPM'}
double Id0x0c08a7f08c08a7f0::fmotspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.500000;
  return ret;
}

// config detail: {'name': 'fmotcur', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|5553.5]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': 'A'}
double Id0x0c08a7f08c08a7f0::fmotcur(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -1000.000000;
  return ret;
}

// config detail: {'name': 'bymcusts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 48, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c08a7f08c08a7f0::bymcusts1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// // config detail: {'name': 'bymcusts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 49, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
// bool Id0x0c08a7f08c08a7f0::bymcusts(const std::uint8_t* bytes, int32_t length) const {
//   Byte t0(bytes + 6);
//   int32_t x = t0.get_byte(1, 1);

//   bool ret = x;
//   return ret;
// }

// // config detail: {'name': 'bymcusts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 50, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
// bool Id0x0c08a7f08c08a7f0::bymcusts(const std::uint8_t* bytes, int32_t length) const {
//   Byte t0(bytes + 6);
//   int32_t x = t0.get_byte(2, 1);

//   bool ret = x;
//   return ret;
// }

// // config detail: {'name': 'bymcusts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 51, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
// bool Id0x0c08a7f08c08a7f0::bymcusts(const std::uint8_t* bytes, int32_t length) const {
//   Byte t0(bytes + 6);
//   int32_t x = t0.get_byte(3, 1);

//   bool ret = x;
//   return ret;
// }

// // config detail: {'name': 'bymcusts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 52, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
// bool Id0x0c08a7f08c08a7f0::bymcusts(const std::uint8_t* bytes, int32_t length) const {
//   Byte t0(bytes + 6);
//   int32_t x = t0.get_byte(4, 1);

//   bool ret = x;
//   return ret;
// }

// // config detail: {'name': 'bymcusts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 53, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
// bool Id0x0c08a7f08c08a7f0::bymcusts(const std::uint8_t* bytes, int32_t length) const {
//   Byte t0(bytes + 6);
//   int32_t x = t0.get_byte(5, 1);

//   bool ret = x;
//   return ret;
// }

// // config detail: {'name': 'bymcusts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 54, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
// bool Id0x0c08a7f08c08a7f0::bymcusts(const std::uint8_t* bytes, int32_t length) const {
//   Byte t0(bytes + 6);
//   int32_t x = t0.get_byte(6, 1);

//   bool ret = x;
//   return ret;
// }

// // config detail: {'name': 'bymcusts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 55, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
// bool Id0x0c08a7f08c08a7f0::bymcusts(const std::uint8_t* bytes, int32_t length) const {
//   Byte t0(bytes + 6);
//   int32_t x = t0.get_byte(7, 1);

//   bool ret = x;
//   return ret;
// }

// config detail: {'name': 'bymcu1msglife1', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Id0x0c08a7f08c08a7f0::bymcu1msglife1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
