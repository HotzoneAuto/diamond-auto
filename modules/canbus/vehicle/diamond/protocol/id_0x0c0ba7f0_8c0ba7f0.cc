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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c0ba7f0_8c0ba7f0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x0c0ba7f08c0ba7f0::Id0x0c0ba7f08c0ba7f0() {}
const int32_t Id0x0c0ba7f08c0ba7f0::ID = 0x2c0ba7f0;

void Id0x0c0ba7f08c0ba7f0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_fmotmaxpower(fmotmaxpower(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_fmotmaxspd(fmotmaxspd(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_fmotmaxtq(fmotmaxtq(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0_8c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
}

// config detail: {'name': 'fmotmaxpower', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'kw'}
int Id0x0c0ba7f08c0ba7f0::fmotmaxpower(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'fmotmaxspd', 'offset': 0.0, 'precision': 100.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|25500]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': 'rpm'}
double Id0x0c0ba7f08c0ba7f0::fmotmaxspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 100.000000;
  return ret;
}

// config detail: {'name': 'fmotmaxtq', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': 'NM'}
int Id0x0c0ba7f08c0ba7f0::fmotmaxtq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 32, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 33, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 34, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 35, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 36, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 37, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 38, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 39, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 40, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 41, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 42, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 43, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'dwmcuerrflg', 'offset': 0.0, 'precision': 1.0, 'len': 20, 'is_signed_var': False, 'physical_range': '[0|1048575]', 'bit': 44, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Id0x0c0ba7f08c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 5);
  t = t2.get_byte(4, 4);
  x <<= 4;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
