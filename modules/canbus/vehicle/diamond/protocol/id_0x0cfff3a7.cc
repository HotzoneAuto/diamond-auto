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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0cfff3a7.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0cfff3a7::ID = 0x0CFFF3A7;

// public
Id0x0cfff3a7::Id0x0cfff3a7() { Reset(); }

uint32_t Id0x0cfff3a7::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0cfff3a7::UpdateData(uint8_t* data) {
  set_p_bybatrlyoffcmd(data, bybatrlyoffcmd_);
  set_p_bybatrlycmd(data, bybatrlycmd_);
}

void Id0x0cfff3a7::Reset() {
  // TODO(All) :  you should check this manually
  bybatrlyoffcmd_ = 0;
  bybatrlycmd_ = 0;
}

Id0x0cfff3a7* Id0x0cfff3a7::set_bybatrlyoffcmd(
    int bybatrlyoffcmd) {
  bybatrlyoffcmd_ = bybatrlyoffcmd;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'byBatRlyOffCmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0cfff3a7::set_p_bybatrlyoffcmd(uint8_t* data,
    int bybatrlyoffcmd) {
  bybatrlyoffcmd = ProtocolData::BoundedValue(0, 255, bybatrlyoffcmd);
  int x = bybatrlyoffcmd;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Id0x0cfff3a7* Id0x0cfff3a7::set_bybatrlycmd(
    int bybatrlycmd) {
  bybatrlycmd_ = bybatrlycmd;
  return this;
 }

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'byBatRlyCmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0cfff3a7::set_p_bybatrlycmd(uint8_t* data,
    int bybatrlycmd) {
  bybatrlycmd = ProtocolData::BoundedValue(0, 255, bybatrlycmd);
  int x = bybatrlycmd;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
