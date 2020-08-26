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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0b19f0a8.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0b19f0a8::ID = 0x0B19F0A8;

// public
Id0x0b19f0a8::Id0x0b19f0a8() { Reset(); }

uint32_t Id0x0b19f0a8::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0b19f0a8::UpdateData(uint8_t* data) {
  set_p_k1_high_low_vol_control(data, k1_high_low_vol_control_);
  set_p_k2_high_low_vol_control(data, k2_high_low_vol_control_);
}

void Id0x0b19f0a8::Reset() {
  // TODO(All) :  you should check this manually
  k1_high_low_vol_control_ = 0;
  k2_high_low_vol_control_ = 0;
}

Id0x0b19f0a8* Id0x0b19f0a8::set_k1_high_low_vol_control(
    int k1_high_low_vol_control) {
  k1_high_low_vol_control_ = k1_high_low_vol_control;
  return this;
 }

// config detail: {'name': 'K1_High_low_Vol_control', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'V'}
void Id0x0b19f0a8::set_p_k1_high_low_vol_control(uint8_t* data,
    int k1_high_low_vol_control) {
  k1_high_low_vol_control = ProtocolData::BoundedValue(0, 10, k1_high_low_vol_control);
  int x = k1_high_low_vol_control;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Id0x0b19f0a8* Id0x0b19f0a8::set_k2_high_low_vol_control(
    int k2_high_low_vol_control) {
  k2_high_low_vol_control_ = k2_high_low_vol_control;
  return this;
 }

// config detail: {'name': 'K2_High_low_Vol_control', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': 'V'}
void Id0x0b19f0a8::set_p_k2_high_low_vol_control(uint8_t* data,
    int k2_high_low_vol_control) {
  k2_high_low_vol_control = ProtocolData::BoundedValue(0, 10, k2_high_low_vol_control);
  int x = k2_high_low_vol_control;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo