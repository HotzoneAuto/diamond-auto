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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c079aa7_8c079aa7.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0c079aa78c079aa7::ID = 0x2c079aa7;

// public
Id0x0c079aa78c079aa7::Id0x0c079aa78c079aa7() { Reset(); }

uint32_t Id0x0c079aa78c079aa7::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0c079aa78c079aa7::UpdateData(uint8_t* data) {
  set_p_bydcdccmd(data, bydcdccmd_);
  set_p_bydcaccmd(data, bydcaccmd_);
  set_p_bydcacwkst(data, bydcacwkst_);
  set_p_byeapcmd(data, byeapcmd_);
  set_p_bydcac2cmd(data, bydcac2cmd_);
  set_p_bydcac2wkst(data, bydcac2wkst_);
}

void Id0x0c079aa78c079aa7::Reset() {
  // TODO(All) :  you should check this manually
  bydcdccmd_ = 0;
  bydcaccmd_ = 0;
  bydcacwkst_ = 0;
  byeapcmd_ = 0;
  bydcac2cmd_ = 0;
  bydcac2wkst_ = 0;
}

Id0x0c079aa78c079aa7* Id0x0c079aa78c079aa7::set_bydcdccmd(
    int bydcdccmd) {
  bydcdccmd_ = bydcdccmd;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'byDcdcCmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c079aa78c079aa7::set_p_bydcdccmd(uint8_t* data,
    int bydcdccmd) {
  bydcdccmd = ProtocolData::BoundedValue(0, 255, bydcdccmd);
  int x = bydcdccmd;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Id0x0c079aa78c079aa7* Id0x0c079aa78c079aa7::set_bydcaccmd(
    int bydcaccmd) {
  bydcaccmd_ = bydcaccmd;
  return this;
 }

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'byDcAcCmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c079aa78c079aa7::set_p_bydcaccmd(uint8_t* data,
    int bydcaccmd) {
  bydcaccmd = ProtocolData::BoundedValue(0, 255, bydcaccmd);
  int x = bydcaccmd;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}


Id0x0c079aa78c079aa7* Id0x0c079aa78c079aa7::set_bydcacwkst(
    int bydcacwkst) {
  bydcacwkst_ = bydcacwkst;
  return this;
 }

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name': 'byDcAcWkSt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c079aa78c079aa7::set_p_bydcacwkst(uint8_t* data,
    int bydcacwkst) {
  bydcacwkst = ProtocolData::BoundedValue(0, 255, bydcacwkst);
  int x = bydcacwkst;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 8);
}


Id0x0c079aa78c079aa7* Id0x0c079aa78c079aa7::set_byeapcmd(
    int byeapcmd) {
  byeapcmd_ = byeapcmd;
  return this;
 }

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'byEapCmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c079aa78c079aa7::set_p_byeapcmd(uint8_t* data,
    int byeapcmd) {
  byeapcmd = ProtocolData::BoundedValue(0, 255, byeapcmd);
  int x = byeapcmd;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}


Id0x0c079aa78c079aa7* Id0x0c079aa78c079aa7::set_bydcac2cmd(
    int bydcac2cmd) {
  bydcac2cmd_ = bydcac2cmd;
  return this;
 }

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name': 'byDcAc2Cmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c079aa78c079aa7::set_p_bydcac2cmd(uint8_t* data,
    int bydcac2cmd) {
  bydcac2cmd = ProtocolData::BoundedValue(0, 255, bydcac2cmd);
  int x = bydcac2cmd;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}


Id0x0c079aa78c079aa7* Id0x0c079aa78c079aa7::set_bydcac2wkst(
    int bydcac2wkst) {
  bydcac2wkst_ = bydcac2wkst;
  return this;
 }

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name': 'byDcAc2WkSt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c079aa78c079aa7::set_p_bydcac2wkst(uint8_t* data,
    int bydcac2wkst) {
  bydcac2wkst = ProtocolData::BoundedValue(0, 255, bydcac2wkst);
  int x = bydcac2wkst;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
