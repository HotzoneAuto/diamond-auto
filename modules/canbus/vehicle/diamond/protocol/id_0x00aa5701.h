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

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace diamond {

class Id0x00aa5701 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x00aa5701();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name': 'Relay1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay1(int relay1);

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 4, 'name': 'Relay2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay2(int relay2);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 4, 'name': 'Relay3', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay3(int relay3);

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 4, 'name': 'Relay4', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay4(int relay4);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 4, 'name': 'Relay5', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay5(int relay5);

  // config detail: {'bit': 20, 'is_signed_var': False, 'len': 4, 'name': 'Relay6', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay6(int relay6);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 4, 'name': 'Relay7', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay7(int relay7);

  // config detail: {'bit': 28, 'is_signed_var': False, 'len': 4, 'name': 'Relay8', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay8(int relay8);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 4, 'name': 'Relay9', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay9(int relay9);

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 4, 'name': 'Relay10', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay10(int relay10);

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 4, 'name': 'Relay11', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay11(int relay11);

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 4, 'name': 'Relay12', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay12(int relay12);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 4, 'name': 'Relay13', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay13(int relay13);

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'Relay14', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay14(int relay14);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 4, 'name': 'Relay15', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay15(int relay15);

  // config detail: {'bit': 60, 'is_signed_var': False, 'len': 4, 'name': 'Relay16', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x00aa5701* set_relay16(int relay16);

 private:

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 4, 'name': 'Relay1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay1(uint8_t* data, int relay1);

  // config detail: {'bit': 4, 'is_signed_var': False, 'len': 4, 'name': 'Relay2', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay2(uint8_t* data, int relay2);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 4, 'name': 'Relay3', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay3(uint8_t* data, int relay3);

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 4, 'name': 'Relay4', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay4(uint8_t* data, int relay4);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 4, 'name': 'Relay5', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay5(uint8_t* data, int relay5);

  // config detail: {'bit': 20, 'is_signed_var': False, 'len': 4, 'name': 'Relay6', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay6(uint8_t* data, int relay6);

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 4, 'name': 'Relay7', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay7(uint8_t* data, int relay7);

  // config detail: {'bit': 28, 'is_signed_var': False, 'len': 4, 'name': 'Relay8', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay8(uint8_t* data, int relay8);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 4, 'name': 'Relay9', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay9(uint8_t* data, int relay9);

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 4, 'name': 'Relay10', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay10(uint8_t* data, int relay10);

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 4, 'name': 'Relay11', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay11(uint8_t* data, int relay11);

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 4, 'name': 'Relay12', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay12(uint8_t* data, int relay12);

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 4, 'name': 'Relay13', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay13(uint8_t* data, int relay13);

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'Relay14', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay14(uint8_t* data, int relay14);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 4, 'name': 'Relay15', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay15(uint8_t* data, int relay15);

  // config detail: {'bit': 60, 'is_signed_var': False, 'len': 4, 'name': 'Relay16', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_relay16(uint8_t* data, int relay16);

 private:
  int relay1_;
  int relay2_;
  int relay3_;
  int relay4_;
  int relay5_;
  int relay6_;
  int relay7_;
  int relay8_;
  int relay9_;
  int relay10_;
  int relay11_;
  int relay12_;
  int relay13_;
  int relay14_;
  int relay15_;
  int relay16_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


