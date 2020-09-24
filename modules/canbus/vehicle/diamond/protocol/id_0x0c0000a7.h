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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace diamond {

class Id0x0c0000a7 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x0c0000a7();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt0_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c0000a7* set_fmt0_send(int fmt0_send);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt1_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c0000a7* set_fmt1_send(int fmt1_send);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt2_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c0000a7* set_fmt2_send(int fmt2_send);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt3_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c0000a7* set_fmt3_send(int fmt3_send);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt4_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c0000a7* set_fmt4_send(int fmt4_send);

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt5_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c0000a7* set_fmt5_send(int fmt5_send);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt6_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c0000a7* set_fmt6_send(int fmt6_send);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'Parking_Mode_send', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c0000a7* set_parking_mode_send(int parking_mode_send);

 private:
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt0_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_fmt0_send(uint8_t* data, int fmt0_send);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt1_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_fmt1_send(uint8_t* data, int fmt1_send);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt2_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_fmt2_send(uint8_t* data, int fmt2_send);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt3_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_fmt3_send(uint8_t* data, int fmt3_send);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt4_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_fmt4_send(uint8_t* data, int fmt4_send);

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt5_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_fmt5_send(uint8_t* data, int fmt5_send);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'Fmt6_send', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_fmt6_send(uint8_t* data, int fmt6_send);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name':
  // 'Parking_Mode_send', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_parking_mode_send(uint8_t* data, int parking_mode_send);

 private:
  int fmt0_send_;
  int fmt1_send_;
  int fmt2_send_;
  int fmt3_send_;
  int fmt4_send_;
  int fmt5_send_;
  int fmt6_send_;
  int parking_mode_send_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
