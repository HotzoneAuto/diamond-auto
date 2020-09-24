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

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c0000a7.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x0c0000a7::ID = 0x0C0000A7;

// public
Id0x0c0000a7::Id0x0c0000a7() { Reset(); }

uint32_t Id0x0c0000a7::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x0c0000a7::UpdateData(uint8_t* data) {
  set_p_fmt0_send(data, fmt0_send_);
  set_p_fmt1_send(data, fmt1_send_);
  set_p_fmt2_send(data, fmt2_send_);
  set_p_fmt3_send(data, fmt3_send_);
  set_p_fmt4_send(data, fmt4_send_);
  set_p_fmt5_send(data, fmt5_send_);
  set_p_fmt6_send(data, fmt6_send_);
  set_p_parking_mode_send(data, parking_mode_send_);
}

void Id0x0c0000a7::Reset() {
  // TODO(All) :  you should check this manually
  fmt0_send_ = 0;
  fmt1_send_ = 0;
  fmt2_send_ = 0;
  fmt3_send_ = 0;
  fmt4_send_ = 0;
  fmt5_send_ = 0;
  fmt6_send_ = 0;
  parking_mode_send_ = 0;
}

Id0x0c0000a7* Id0x0c0000a7::set_fmt0_send(int fmt0_send) {
  fmt0_send_ = fmt0_send;
  return this;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
// 'Fmt0_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c0000a7::set_p_fmt0_send(uint8_t* data, int fmt0_send) {
  fmt0_send = ProtocolData::BoundedValue(0, 0, fmt0_send);
  int x = fmt0_send;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

Id0x0c0000a7* Id0x0c0000a7::set_fmt1_send(int fmt1_send) {
  fmt1_send_ = fmt1_send;
  return this;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
// 'Fmt1_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c0000a7::set_p_fmt1_send(uint8_t* data, int fmt1_send) {
  fmt1_send = ProtocolData::BoundedValue(0, 0, fmt1_send);
  int x = fmt1_send;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}

Id0x0c0000a7* Id0x0c0000a7::set_fmt2_send(int fmt2_send) {
  fmt2_send_ = fmt2_send;
  return this;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
// 'Fmt2_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c0000a7::set_p_fmt2_send(uint8_t* data, int fmt2_send) {
  fmt2_send = ProtocolData::BoundedValue(0, 0, fmt2_send);
  int x = fmt2_send;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}

Id0x0c0000a7* Id0x0c0000a7::set_fmt3_send(int fmt3_send) {
  fmt3_send_ = fmt3_send;
  return this;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
// 'Fmt3_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c0000a7::set_p_fmt3_send(uint8_t* data, int fmt3_send) {
  fmt3_send = ProtocolData::BoundedValue(0, 0, fmt3_send);
  int x = fmt3_send;

  Byte to_set(data + 3);
  to_set.set_value(x, 0, 8);
}

Id0x0c0000a7* Id0x0c0000a7::set_fmt4_send(int fmt4_send) {
  fmt4_send_ = fmt4_send;
  return this;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
// 'Fmt4_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c0000a7::set_p_fmt4_send(uint8_t* data, int fmt4_send) {
  fmt4_send = ProtocolData::BoundedValue(0, 0, fmt4_send);
  int x = fmt4_send;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}

Id0x0c0000a7* Id0x0c0000a7::set_fmt5_send(int fmt5_send) {
  fmt5_send_ = fmt5_send;
  return this;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
// 'Fmt5_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c0000a7::set_p_fmt5_send(uint8_t* data, int fmt5_send) {
  fmt5_send = ProtocolData::BoundedValue(0, 0, fmt5_send);
  int x = fmt5_send;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}

Id0x0c0000a7* Id0x0c0000a7::set_fmt6_send(int fmt6_send) {
  fmt6_send_ = fmt6_send;
  return this;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
// 'Fmt6_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c0000a7::set_p_fmt6_send(uint8_t* data, int fmt6_send) {
  fmt6_send = ProtocolData::BoundedValue(0, 0, fmt6_send);
  int x = fmt6_send;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}

Id0x0c0000a7* Id0x0c0000a7::set_parking_mode_send(int parking_mode_send) {
  parking_mode_send_ = parking_mode_send;
  return this;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
// 'Parking_Mode_send', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Id0x0c0000a7::set_p_parking_mode_send(uint8_t* data,
                                           int parking_mode_send) {
  parking_mode_send = ProtocolData::BoundedValue(0, 1, parking_mode_send);
  int x = parking_mode_send;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
