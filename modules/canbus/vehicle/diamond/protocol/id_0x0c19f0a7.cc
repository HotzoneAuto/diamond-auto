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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c19f0a7.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0c19f0a7::ID = 0x0C19F0A7;

// public
Id0x0c19f0a7::Id0x0c19f0a7() { Reset(); }

uint32_t Id0x0c19f0a7::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0c19f0a7::UpdateData(uint8_t* data) {
  set_p_fmot1targettq(data, fmot1targettq_);
  set_p_fmot1lmtvolt(data, fmot1lmtvolt_);
  set_p_fmot1lmtcur(data, fmot1lmtcur_);
  set_p_bymot1workmode(data, bymot1workmode_);
  set_p_bylife(data, bylife_);
}

void Id0x0c19f0a7::Reset() {
  fmot1targettq_ = 0;
  fmot1lmtvolt_ = 800;
  fmot1lmtcur_ = 250;
  // default motor speed mode
  bymot1workmode_ = 129;
  bylife_ = 0;
}

Id0x0c19f0a7* Id0x0c19f0a7::set_fmot1targettq(int fmot1targettq) {
  fmot1targettq_ = fmot1targettq;
  return this;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
// 'fMot1TargetTq', 'offset': -32000.0, 'order': 'intel', 'physical_range':
// '[-32000|33535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
void Id0x0c19f0a7::set_p_fmot1targettq(uint8_t* data, int fmot1targettq) {
  fmot1targettq = ProtocolData::BoundedValue(-32000, 33535, fmot1targettq);
  int x = (fmot1targettq - -32000.000000);
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

Id0x0c19f0a7* Id0x0c19f0a7::set_fmot1lmtvolt(double fmot1lmtvolt) {
  fmot1lmtvolt_ = fmot1lmtvolt;
  return this;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name':
// 'fMot1LmtVolt', 'offset': -1000.0, 'order': 'intel', 'physical_range':
// '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
void Id0x0c19f0a7::set_p_fmot1lmtvolt(uint8_t* data, double fmot1lmtvolt) {
  fmot1lmtvolt = ProtocolData::BoundedValue(-1000.0, 5553.5, fmot1lmtvolt);
  int x = (fmot1lmtvolt - -1000.000000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}

Id0x0c19f0a7* Id0x0c19f0a7::set_fmot1lmtcur(double fmot1lmtcur) {
  fmot1lmtcur_ = fmot1lmtcur;
  return this;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name':
// 'fMot1LmtCur', 'offset': -1000.0, 'order': 'intel', 'physical_range':
// '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
void Id0x0c19f0a7::set_p_fmot1lmtcur(uint8_t* data, double fmot1lmtcur) {
  fmot1lmtcur = ProtocolData::BoundedValue(-1000.0, 5553.5, fmot1lmtcur);
  int x = (fmot1lmtcur - -1000.000000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}

Id0x0c19f0a7* Id0x0c19f0a7::set_bymot1workmode(int bymot1workmode) {
  bymot1workmode_ = bymot1workmode;
  return this;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
// 'byMot1WorkMode', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c19f0a7::set_p_bymot1workmode(uint8_t* data, int bymot1workmode) {
  bymot1workmode = ProtocolData::BoundedValue(0, 255, bymot1workmode);
  int x = bymot1workmode;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}

Id0x0c19f0a7* Id0x0c19f0a7::set_bylife(int bylife) {
  bylife_ = bylife;
  return this;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'byLife', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c19f0a7::set_p_bylife(uint8_t* data, int bylife) {
  bylife = ProtocolData::BoundedValue(0, 255, bylife);
  int x = bylife;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
