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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c19f0a7_8c19f0a7.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0c19f0a78c19f0a7::ID = 0x2c19f0a7;

// public
Id0x0c19f0a78c19f0a7::Id0x0c19f0a78c19f0a7() { Reset(); }

uint32_t Id0x0c19f0a78c19f0a7::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0c19f0a78c19f0a7::UpdateData(uint8_t* data) {
  set_p_fmot1targettq(data, fmot1targettq_);
  set_p_fmot1lmtvolt(data, fmot1lmtvolt_);
  set_p_fmot1lmtcur(data, fmot1lmtcur_);
  set_p_bymot1workmode(data, bymot1workmode_);
  set_p_bylife(data, bylife_);
}

void Id0x0c19f0a78c19f0a7::Reset() {
  // TODO(All) :  you should check this manually
  fmot1targettq_ = 0;
  fmot1lmtvolt_ = 0.0;
  fmot1lmtcur_ = 0.0;
  bymot1workmode_ = false;
  bylife_ = 0;
}

Id0x0c19f0a78c19f0a7* Id0x0c19f0a78c19f0a7::set_fmot1targettq(
    int fmot1targettq) {
  fmot1targettq_ = fmot1targettq;
  return this;
 }

// config detail: {'name': 'fMot1TargetTq', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0x0c19f0a78c19f0a7::set_p_fmot1targettq(uint8_t* data,
    int fmot1targettq) {
  fmot1targettq = ProtocolData::BoundedValue(0, 65535, fmot1targettq);
  int x = fmot1targettq;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}


Id0x0c19f0a78c19f0a7* Id0x0c19f0a78c19f0a7::set_fmot1lmtvolt(
    double fmot1lmtvolt) {
  fmot1lmtvolt_ = fmot1lmtvolt;
  return this;
 }

// config detail: {'name': 'fMot1LmtVolt', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|5553.5]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'V'}
void Id0x0c19f0a78c19f0a7::set_p_fmot1lmtvolt(uint8_t* data,
    double fmot1lmtvolt) {
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


Id0x0c19f0a78c19f0a7* Id0x0c19f0a78c19f0a7::set_fmot1lmtcur(
    double fmot1lmtcur) {
  fmot1lmtcur_ = fmot1lmtcur;
  return this;
 }

// config detail: {'name': 'fMot1LmtCur', 'offset': -1000.0, 'precision': 0.1, 'len': 16, 'is_signed_var': False, 'physical_range': '[-1000|5553.5]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': 'A'}
void Id0x0c19f0a78c19f0a7::set_p_fmot1lmtcur(uint8_t* data,
    double fmot1lmtcur) {
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


Id0x0c19f0a78c19f0a7* Id0x0c19f0a78c19f0a7::set_bymot1workmode(
    bool bymot1workmode) {
  bymot1workmode_ = bymot1workmode;
  return this;
 }

// config detail: {'name': 'byMot1WorkMode', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 48, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Id0x0c19f0a78c19f0a7::set_p_bymot1workmode(uint8_t* data,
    bool bymot1workmode) {
  int x = bymot1workmode;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 1);
}


Id0x0c19f0a78c19f0a7* Id0x0c19f0a78c19f0a7::set_bylife(
    int bylife) {
  bylife_ = bylife;
  return this;
 }

// config detail: {'name': 'byLife', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0x0c19f0a78c19f0a7::set_p_bylife(uint8_t* data,
    int bylife) {
  bylife = ProtocolData::BoundedValue(0, 255, bylife);
  int x = bylife;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
