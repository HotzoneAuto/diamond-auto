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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c08a7f0_8c08a7f0.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0c08a7f08c08a7f0::ID = 0x2c08a7f0;

// public
Id0x0c08a7f08c08a7f0::Id0x0c08a7f08c08a7f0() { Reset(); }

uint32_t Id0x0c08a7f08c08a7f0::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0c08a7f08c08a7f0::UpdateData(uint8_t* data) {
  set_p_fmottq(data, fmottq_);
  set_p_fmotspd(data, fmotspd_);
  set_p_fmotcur(data, fmotcur_);
  set_p_bymcusts(data, bymcusts_);
  set_p_bymcusts(data, bymcusts_);
  set_p_bymcusts(data, bymcusts_);
  set_p_bymcusts(data, bymcusts_);
  set_p_bymcusts(data, bymcusts_);
  set_p_bymcusts(data, bymcusts_);
  set_p_bymcusts(data, bymcusts_);
  set_p_bymcusts(data, bymcusts_);
  set_p_bymcu1msglife1(data, bymcu1msglife1_);
}

void Id0x0c08a7f08c08a7f0::Reset() {
  // TODO(All) :  you should check this manually
  fmottq_ = 0;
  fmotspd_ = 0.0;
  fmotcur_ = 0.0;
  bymcusts_ = false;
  bymcusts_ = false;
  bymcusts_ = false;
  bymcusts_ = false;
  bymcusts_ = false;
  bymcusts_ = false;
  bymcusts_ = false;
  bymcusts_ = false;
  bymcu1msglife1_ = 0;
}

Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_fmottq(
    int fmottq) {
  fmottq_ = fmottq;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'fMotTq', 'offset': -32000.0, 'order': 'intel', 'physical_range': '[-32000|33535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
void Id0x0c08a7f08c08a7f0::set_p_fmottq(uint8_t* data,
    int fmottq) {
  fmottq = ProtocolData::BoundedValue(-32000, 33535, fmottq);
  int x = (fmottq - -32000.000000);
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_fmotspd(
    double fmotspd) {
  fmotspd_ = fmotspd;
  return this;
 }

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'fMotSpd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|32767.5]', 'physical_unit': 'RPM', 'precision': 0.5, 'type': 'double'}
void Id0x0c08a7f08c08a7f0::set_p_fmotspd(uint8_t* data,
    double fmotspd) {
  fmotspd = ProtocolData::BoundedValue(0.0, 32767.5, fmotspd);
  int x = fmotspd / 0.500000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_fmotcur(
    double fmotcur) {
  fmotcur_ = fmotcur;
  return this;
 }

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'fMotCur', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
void Id0x0c08a7f08c08a7f0::set_p_fmotcur(uint8_t* data,
    double fmotcur) {
  fmotcur = ProtocolData::BoundedValue(-1000.0, 5553.5, fmotcur);
  int x = (fmotcur - -1000.000000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_bymcusts(
    bool bymcusts) {
  bymcusts_ = bymcusts;
  return this;
 }

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c08a7f08c08a7f0::set_p_bymcusts(uint8_t* data,
    bool bymcusts) {
  int x = bymcusts;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 1);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_bymcusts(
    bool bymcusts) {
  bymcusts_ = bymcusts;
  return this;
 }

// config detail: {'bit': 49, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c08a7f08c08a7f0::set_p_bymcusts(uint8_t* data,
    bool bymcusts) {
  int x = bymcusts;

  Byte to_set(data + 6);
  to_set.set_value(x, 1, 1);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_bymcusts(
    bool bymcusts) {
  bymcusts_ = bymcusts;
  return this;
 }

// config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c08a7f08c08a7f0::set_p_bymcusts(uint8_t* data,
    bool bymcusts) {
  int x = bymcusts;

  Byte to_set(data + 6);
  to_set.set_value(x, 2, 1);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_bymcusts(
    bool bymcusts) {
  bymcusts_ = bymcusts;
  return this;
 }

// config detail: {'bit': 51, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c08a7f08c08a7f0::set_p_bymcusts(uint8_t* data,
    bool bymcusts) {
  int x = bymcusts;

  Byte to_set(data + 6);
  to_set.set_value(x, 3, 1);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_bymcusts(
    bool bymcusts) {
  bymcusts_ = bymcusts;
  return this;
 }

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c08a7f08c08a7f0::set_p_bymcusts(uint8_t* data,
    bool bymcusts) {
  int x = bymcusts;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 1);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_bymcusts(
    bool bymcusts) {
  bymcusts_ = bymcusts;
  return this;
 }

// config detail: {'bit': 53, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c08a7f08c08a7f0::set_p_bymcusts(uint8_t* data,
    bool bymcusts) {
  int x = bymcusts;

  Byte to_set(data + 6);
  to_set.set_value(x, 5, 1);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_bymcusts(
    bool bymcusts) {
  bymcusts_ = bymcusts;
  return this;
 }

// config detail: {'bit': 54, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c08a7f08c08a7f0::set_p_bymcusts(uint8_t* data,
    bool bymcusts) {
  int x = bymcusts;

  Byte to_set(data + 6);
  to_set.set_value(x, 6, 1);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_bymcusts(
    bool bymcusts) {
  bymcusts_ = bymcusts;
  return this;
 }

// config detail: {'bit': 55, 'is_signed_var': False, 'len': 1, 'name': 'byMCUSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c08a7f08c08a7f0::set_p_bymcusts(uint8_t* data,
    bool bymcusts) {
  int x = bymcusts;

  Byte to_set(data + 6);
  to_set.set_value(x, 7, 1);
}


Id0x0c08a7f08c08a7f0* Id0x0c08a7f08c08a7f0::set_bymcu1msglife1(
    int bymcu1msglife1) {
  bymcu1msglife1_ = bymcu1msglife1;
  return this;
 }

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'byMCU1MsgLife1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c08a7f08c08a7f0::set_p_bymcu1msglife1(uint8_t* data,
    int bymcu1msglife1) {
  bymcu1msglife1 = ProtocolData::BoundedValue(0, 255, bymcu1msglife1);
  int x = bymcu1msglife1;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
