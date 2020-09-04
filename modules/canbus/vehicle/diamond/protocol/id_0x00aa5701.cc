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

#include "modules/canbus/vehicle/diamond/protocol/id_0x00aa5701.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x00aa5701::ID = 0x00AA5701;

// public
Id0x00aa5701::Id0x00aa5701() { Reset(); }

uint32_t Id0x00aa5701::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x00aa5701::UpdateData(uint8_t* data) {
  set_p_relay1(data, relay1_);
  set_p_relay2(data, relay2_);
  set_p_relay3(data, relay3_);
  set_p_relay4(data, relay4_);
  set_p_relay5(data, relay5_);
  set_p_relay6(data, relay6_);
  set_p_relay7(data, relay7_);
  set_p_relay8(data, relay8_);
  set_p_relay9(data, relay9_);
  set_p_relay10(data, relay10_);
  set_p_relay11(data, relay11_);
  set_p_relay12(data, relay12_);
  set_p_relay13(data, relay13_);
  set_p_relay14(data, relay14_);
  set_p_relay15(data, relay15_);
  set_p_relay16(data, relay16_);
}

void Id0x00aa5701::Reset() {
  relay1_ = 0;
  relay2_ = 0;
  relay3_ = 0;
  relay4_ = 0;
  relay5_ = 0;
  relay6_ = 0;
  relay7_ = 0;
  relay8_ = 0;
  relay9_ = 0;
  relay10_ = 0;
  relay11_ = 0;
  relay12_ = 0;
  relay13_ = 0;
  relay14_ = 0;
  relay15_ = 0;
  relay16_ = 0;
}

Id0x00aa5701* Id0x00aa5701::set_relay1(int relay1) {
  relay1_ = relay1;
  return this;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name': 'Relay1',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay1(uint8_t* data, int relay1) {
  relay1 = ProtocolData::BoundedValue(0, 1, relay1);
  int x = relay1;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay2(int relay2) {
  relay2_ = relay2;
  return this;
}

// config detail: {'bit': 4, 'is_signed_var': False, 'len': 4, 'name': 'Relay2',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay2(uint8_t* data, int relay2) {
  relay2 = ProtocolData::BoundedValue(0, 1, relay2);
  int x = relay2;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay3(int relay3) {
  relay3_ = relay3;
  return this;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 4, 'name': 'Relay3',
// 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit':
// '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay3(uint8_t* data, int relay3) {
  relay3 = ProtocolData::BoundedValue(0, 1, relay3);
  int x = relay3;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay4(int relay4) {
  relay4_ = relay4;
  return this;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay4', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay4(uint8_t* data, int relay4) {
  relay4 = ProtocolData::BoundedValue(0, 1, relay4);
  int x = relay4;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay5(int relay5) {
  relay5_ = relay5;
  return this;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay5', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay5(uint8_t* data, int relay5) {
  relay5 = ProtocolData::BoundedValue(0, 1, relay5);
  int x = relay5;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay6(int relay6) {
  relay6_ = relay6;
  return this;
}

// config detail: {'bit': 20, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay6', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay6(uint8_t* data, int relay6) {
  relay6 = ProtocolData::BoundedValue(0, 1, relay6);
  int x = relay6;

  Byte to_set(data + 2);
  to_set.set_value(x, 4, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay7(int relay7) {
  relay7_ = relay7;
  return this;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay7', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay7(uint8_t* data, int relay7) {
  relay7 = ProtocolData::BoundedValue(0, 1, relay7);
  int x = relay7;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay8(int relay8) {
  relay8_ = relay8;
  return this;
}

// config detail: {'bit': 28, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay8', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay8(uint8_t* data, int relay8) {
  relay8 = ProtocolData::BoundedValue(0, 1, relay8);
  int x = relay8;

  Byte to_set(data + 3);
  to_set.set_value(x, 4, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay9(int relay9) {
  relay9_ = relay9;
  return this;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay9', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay9(uint8_t* data, int relay9) {
  relay9 = ProtocolData::BoundedValue(0, 1, relay9);
  int x = relay9;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay10(int relay10) {
  relay10_ = relay10;
  return this;
}

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay10', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay10(uint8_t* data, int relay10) {
  relay10 = ProtocolData::BoundedValue(0, 1, relay10);
  int x = relay10;

  Byte to_set(data + 4);
  to_set.set_value(x, 4, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay11(int relay11) {
  relay11_ = relay11;
  return this;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay11', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay11(uint8_t* data, int relay11) {
  relay11 = ProtocolData::BoundedValue(0, 1, relay11);
  int x = relay11;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay12(int relay12) {
  relay12_ = relay12;
  return this;
}

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay12', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay12(uint8_t* data, int relay12) {
  relay12 = ProtocolData::BoundedValue(0, 1, relay12);
  int x = relay12;

  Byte to_set(data + 5);
  to_set.set_value(x, 4, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay13(int relay13) {
  relay13_ = relay13;
  return this;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay13', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay13(uint8_t* data, int relay13) {
  relay13 = ProtocolData::BoundedValue(0, 1, relay13);
  int x = relay13;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay14(int relay14) {
  relay14_ = relay14;
  return this;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay14', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay14(uint8_t* data, int relay14) {
  relay14 = ProtocolData::BoundedValue(0, 1, relay14);
  int x = relay14;

  Byte to_set(data + 6);
  to_set.set_value(x, 4, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay15(int relay15) {
  relay15_ = relay15;
  return this;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay15', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay15(uint8_t* data, int relay15) {
  relay15 = ProtocolData::BoundedValue(0, 1, relay15);
  int x = relay15;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 4);
}

Id0x00aa5701* Id0x00aa5701::set_relay16(int relay16) {
  relay16_ = relay16;
  return this;
}

// config detail: {'bit': 60, 'is_signed_var': False, 'len': 4, 'name':
// 'Relay16', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x00aa5701::set_p_relay16(uint8_t* data, int relay16) {
  relay16 = ProtocolData::BoundedValue(0, 1, relay16);
  int x = relay16;

  Byte to_set(data + 7);
  to_set.set_value(x, 4, 4);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
