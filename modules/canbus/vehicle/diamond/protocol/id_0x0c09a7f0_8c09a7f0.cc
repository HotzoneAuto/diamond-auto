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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c09a7f0_8c09a7f0.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0c09a7f08c09a7f0::ID = 0x2c09a7f0;

// public
Id0x0c09a7f08c09a7f0::Id0x0c09a7f08c09a7f0() { Reset(); }

uint32_t Id0x0c09a7f08c09a7f0::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0c09a7f08c09a7f0::UpdateData(uint8_t* data) {
  set_p_fmotvolt(data, fmotvolt_);
  set_p_fmottemp(data, fmottemp_);
  set_p_fmcutemp(data, fmcutemp_);
  set_p_fmotrectcur(data, fmotrectcur_);
  set_p_bymotspddir(data, bymotspddir_);
  set_p_bymoterrcode(data, bymoterrcode_);
}

void Id0x0c09a7f08c09a7f0::Reset() {
  // TODO(All) :  you should check this manually
  fmotvolt_ = 0.0;
  fmottemp_ = 0;
  fmcutemp_ = 0;
  fmotrectcur_ = 0.0;
  bymotspddir_ = false;
  bymoterrcode_ = 0;
}

Id0x0c09a7f08c09a7f0* Id0x0c09a7f08c09a7f0::set_fmotvolt(
    double fmotvolt) {
  fmotvolt_ = fmotvolt;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'fMotVolt', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
void Id0x0c09a7f08c09a7f0::set_p_fmotvolt(uint8_t* data,
    double fmotvolt) {
  fmotvolt = ProtocolData::BoundedValue(-1000.0, 5553.5, fmotvolt);
  int x = (fmotvolt - -1000.000000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}


Id0x0c09a7f08c09a7f0* Id0x0c09a7f08c09a7f0::set_fmottemp(
    int fmottemp) {
  fmottemp_ = fmottemp;
  return this;
 }

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'fMotTemp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|215]', 'physical_unit': 'TMP', 'precision': 1.0, 'type': 'int'}
void Id0x0c09a7f08c09a7f0::set_p_fmottemp(uint8_t* data,
    int fmottemp) {
  fmottemp = ProtocolData::BoundedValue(-40, 215, fmottemp);
  int x = (fmottemp - -40.000000);

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}


Id0x0c09a7f08c09a7f0* Id0x0c09a7f08c09a7f0::set_fmcutemp(
    int fmcutemp) {
  fmcutemp_ = fmcutemp;
  return this;
 }

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'fMCUTemp', 'offset': -40.0, 'order': 'intel', 'physical_range': '[-40|215]', 'physical_unit': 'TMP', 'precision': 1.0, 'type': 'int'}
void Id0x0c09a7f08c09a7f0::set_p_fmcutemp(uint8_t* data,
    int fmcutemp) {
  fmcutemp = ProtocolData::BoundedValue(-40, 215, fmcutemp);
  int x = (fmcutemp - -40.000000);

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 8);
}


Id0x0c09a7f08c09a7f0* Id0x0c09a7f08c09a7f0::set_fmotrectcur(
    double fmotrectcur) {
  fmotrectcur_ = fmotrectcur;
  return this;
 }

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 16, 'name': 'fMotRectCur', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
void Id0x0c09a7f08c09a7f0::set_p_fmotrectcur(uint8_t* data,
    double fmotrectcur) {
  fmotrectcur = ProtocolData::BoundedValue(-1000.0, 5553.5, fmotrectcur);
  int x = (fmotrectcur - -1000.000000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}


Id0x0c09a7f08c09a7f0* Id0x0c09a7f08c09a7f0::set_bymotspddir(
    bool bymotspddir) {
  bymotspddir_ = bymotspddir;
  return this;
 }

// config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name': 'byMotSpdDir', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c09a7f08c09a7f0::set_p_bymotspddir(uint8_t* data,
    bool bymotspddir) {
  int x = bymotspddir;

  Byte to_set(data + 6);
  to_set.set_value(x, 2, 1);
}


Id0x0c09a7f08c09a7f0* Id0x0c09a7f08c09a7f0::set_bymoterrcode(
    int bymoterrcode) {
  bymoterrcode_ = bymoterrcode;
  return this;
 }

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'byMotErrCode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c09a7f08c09a7f0::set_p_bymoterrcode(uint8_t* data,
    int bymoterrcode) {
  bymoterrcode = ProtocolData::BoundedValue(0, 255, bymoterrcode);
  int x = bymoterrcode;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
