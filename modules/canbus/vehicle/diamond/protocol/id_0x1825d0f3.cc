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

#include "modules/canbus/vehicle/diamond/protocol/id_0x1825d0f3.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x1825d0f3::Id0x1825d0f3() {}
const int32_t Id0x1825d0f3::ID = 0x1825D0F3;

void Id0x1825d0f3::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  auto id = chassis->mutable_diamond()->mutable_id_0x1825d0f3();
  id->set_fbatmaxcellvolt(fbatmaxcellvolt(bytes, length));
  id->set_fbatmincellvolt(fbatmincellvolt(bytes, length));
  id->set_fbatavrcellvolt(fbatavrcellvolt(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
// 'fbatmaxcellvolt', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|65535]', 'physical_unit': 'mV', 'precision': 1.0, 'type': 'int'}
int Id0x1825d0f3::fbatmaxcellvolt(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name':
// 'fbatmincellvolt', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|65535]', 'physical_unit': 'mV', 'precision': 1.0, 'type': 'int'}
int Id0x1825d0f3::fbatmincellvolt(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name':
// 'fbatavrcellvolt', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|65535]', 'physical_unit': 'mV', 'precision': 1.0, 'type': 'int'}
int Id0x1825d0f3::fbatavrcellvolt(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
