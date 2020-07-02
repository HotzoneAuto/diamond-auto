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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c0ba7f0_8c0ba7f0.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0c0ba7f08c0ba7f0::ID = 0x2c0ba7f0;

// public
Id0x0c0ba7f08c0ba7f0::Id0x0c0ba7f08c0ba7f0() { Reset(); }

uint32_t Id0x0c0ba7f08c0ba7f0::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0c0ba7f08c0ba7f0::UpdateData(uint8_t* data) {
  set_p_fmotmaxpower(data, fmotmaxpower_);
  set_p_fmotmaxspd(data, fmotmaxspd_);
  set_p_fmotmaxtq(data, fmotmaxtq_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
  set_p_dwmcuerrflg(data, dwmcuerrflg_);
}

void Id0x0c0ba7f08c0ba7f0::Reset() {
  // TODO(All) :  you should check this manually
  fmotmaxpower_ = 0;
  fmotmaxspd_ = 0.0;
  fmotmaxtq_ = 0;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = false;
  dwmcuerrflg_ = 0;
}

Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_fmotmaxpower(
    int fmotmaxpower) {
  fmotmaxpower_ = fmotmaxpower;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'fMotMaxPower', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': 'kw', 'precision': 1.0, 'type': 'int'}
void Id0x0c0ba7f08c0ba7f0::set_p_fmotmaxpower(uint8_t* data,
    int fmotmaxpower) {
  fmotmaxpower = ProtocolData::BoundedValue(0, 255, fmotmaxpower);
  int x = fmotmaxpower;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_fmotmaxspd(
    double fmotmaxspd) {
  fmotmaxspd_ = fmotmaxspd;
  return this;
 }

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'fMotMaxSpd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25500]', 'physical_unit': 'rpm', 'precision': 100.0, 'type': 'double'}
void Id0x0c0ba7f08c0ba7f0::set_p_fmotmaxspd(uint8_t* data,
    double fmotmaxspd) {
  fmotmaxspd = ProtocolData::BoundedValue(0.0, 25500.0, fmotmaxspd);
  int x = fmotmaxspd / 100.000000;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_fmotmaxtq(
    int fmotmaxtq) {
  fmotmaxtq_ = fmotmaxtq;
  return this;
 }

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'fMotMaxTq', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
void Id0x0c0ba7f08c0ba7f0::set_p_fmotmaxtq(uint8_t* data,
    int fmotmaxtq) {
  fmotmaxtq = ProtocolData::BoundedValue(0, 65535, fmotmaxtq);
  int x = fmotmaxtq;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 4);
  to_set.set_value(x, 1, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 4);
  to_set.set_value(x, 2, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 35, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 4);
  to_set.set_value(x, 3, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 4);
  to_set.set_value(x, 4, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 37, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 4);
  to_set.set_value(x, 5, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 38, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 4);
  to_set.set_value(x, 6, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 4);
  to_set.set_value(x, 7, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 41, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 5);
  to_set.set_value(x, 1, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 42, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 5);
  to_set.set_value(x, 2, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    bool dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 43, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    bool dwmcuerrflg) {
  int x = dwmcuerrflg;

  Byte to_set(data + 5);
  to_set.set_value(x, 3, 1);
}


Id0x0c0ba7f08c0ba7f0* Id0x0c0ba7f08c0ba7f0::set_dwmcuerrflg(
    int dwmcuerrflg) {
  dwmcuerrflg_ = dwmcuerrflg;
  return this;
 }

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 20, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1048575]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c0ba7f08c0ba7f0::set_p_dwmcuerrflg(uint8_t* data,
    int dwmcuerrflg) {
  dwmcuerrflg = ProtocolData::BoundedValue(0, 1048575, dwmcuerrflg);
  int x = dwmcuerrflg;
  uint8_t t = 0;

  t = x & 0xF;
  Byte to_set0(data + 5);
  to_set0.set_value(t, 4, 4);
  x >>= 4;

  t = x & 0xFF;
  Byte to_set1(data + 6);
  to_set1.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set2(data + 7);
  to_set2.set_value(t, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
