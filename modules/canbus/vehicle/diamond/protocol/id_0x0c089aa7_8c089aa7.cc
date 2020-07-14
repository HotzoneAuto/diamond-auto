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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c089aa7_8c089aa7.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0c089aa78c089aa7::ID = 0x2c089aa7;

// public
Id0x0c089aa78c089aa7::Id0x0c089aa78c089aa7() { Reset(); }

uint32_t Id0x0c089aa78c089aa7::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0c089aa78c089aa7::UpdateData(uint8_t* data) {
  set_p_fdcactargspd(data, fdcactargspd_);
  set_p_bydcacdir(data, bydcacdir_);
  set_p_fdcac2targspd(data, fdcac2targspd_);
  set_p_bydcac2dir(data, bydcac2dir_);
}

void Id0x0c089aa78c089aa7::Reset() {
  // TODO(All) :  you should check this manually
  fdcactargspd_ = 0;
  bydcacdir_ = 0;
  fdcac2targspd_ = 0;
  bydcac2dir_ = 0;
}

Id0x0c089aa78c089aa7* Id0x0c089aa78c089aa7::set_fdcactargspd(
    int fdcactargspd) {
  fdcactargspd_ = fdcactargspd;
  return this;
 }

// config detail: {'name': 'fDcAcTargSpd', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'rpm'}
void Id0x0c089aa78c089aa7::set_p_fdcactargspd(uint8_t* data,
    int fdcactargspd) {
  fdcactargspd = ProtocolData::BoundedValue(0, 65535, fdcactargspd);
  int x = fdcactargspd;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}


Id0x0c089aa78c089aa7* Id0x0c089aa78c089aa7::set_bydcacdir(
    int bydcacdir) {
  bydcacdir_ = bydcacdir;
  return this;
 }

// config detail: {'name': 'byDcAcDir', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0x0c089aa78c089aa7::set_p_bydcacdir(uint8_t* data,
    int bydcacdir) {
  bydcacdir = ProtocolData::BoundedValue(0, 255, bydcacdir);
  int x = bydcacdir;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}


Id0x0c089aa78c089aa7* Id0x0c089aa78c089aa7::set_fdcac2targspd(
    int fdcac2targspd) {
  fdcac2targspd_ = fdcac2targspd;
  return this;
 }

// config detail: {'name': 'fDcAc2TargSpd', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': 'rpm'}
void Id0x0c089aa78c089aa7::set_p_fdcac2targspd(uint8_t* data,
    int fdcac2targspd) {
  fdcac2targspd = ProtocolData::BoundedValue(0, 65535, fdcac2targspd);
  int x = fdcac2targspd;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 3);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 4);
  to_set1.set_value(t, 0, 8);
}


Id0x0c089aa78c089aa7* Id0x0c089aa78c089aa7::set_bydcac2dir(
    int bydcac2dir) {
  bydcac2dir_ = bydcac2dir;
  return this;
 }

// config detail: {'name': 'byDcAc2Dir', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0x0c089aa78c089aa7::set_p_bydcac2dir(uint8_t* data,
    int bydcac2dir) {
  bydcac2dir = ProtocolData::BoundedValue(0, 255, bydcac2dir);
  int x = bydcac2dir;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
