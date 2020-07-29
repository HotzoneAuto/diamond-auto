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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c0ba7f0.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x0c0ba7f0::Id0x0c0ba7f0() {}
const int32_t Id0x0c0ba7f0::ID = 0x0C0BA7F0;

void Id0x0c0ba7f0::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0()->set_fmotmaxpower(fmotmaxpower(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0()->set_fmotmaxspd(fmotmaxspd(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0()->set_fmotmaxtq(fmotmaxtq(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x0c0ba7f0()->set_dwmcuerrflg(dwmcuerrflg(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'fmotmaxpower', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': 'kw', 'precision': 1.0, 'type': 'int'}
int Id0x0c0ba7f0::fmotmaxpower(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'fmotmaxspd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25500]', 'physical_unit': 'rpm', 'precision': 100.0, 'type': 'double'}
double Id0x0c0ba7f0::fmotmaxspd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 100.000000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'fmotmaxtq', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
int Id0x0c0ba7f0::fmotmaxtq(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 32, 'name': 'dwmcuerrflg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|4294967295]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x0c0ba7f0::dwmcuerrflg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 5);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t3(bytes + 4);
  t = t3.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
