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

#include "modules/canbus/vehicle/diamond/protocol/id_0x01.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x01::Id0x01() {}
const int32_t Id0x01::ID = 0x01;

void Id0x01::Parse(const std::uint8_t* bytes, int32_t length,
                   ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x01()->set_angle_sensor_length(
      angle_sensor_length(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x01()->set_angle_sensor_id(
      angle_sensor_id(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x01()->set_angle_sensor_mode(
      angle_sensor_mode(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x01()->set_angle_sensor_data(
      angle_sensor_data(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
// 'angle_sensor_length', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x01::angle_sensor_length(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
// 'angle_sensor_id', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x01::angle_sensor_id(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
// 'angle_sensor_mode', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|8]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x01::angle_sensor_mode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 16, 'name':
// 'angle_sensor_data', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|360]', 'physical_unit': 'deg', 'precision': 0.010986, 'type': 'double'}
double Id0x01::angle_sensor_data(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010986;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
