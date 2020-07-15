/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

class Id0x0c079aa78c079aa7 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x0c079aa78c079aa7();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'byDcdcCmd', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0x0c079aa78c079aa7* set_bydcdccmd(int bydcdccmd);

  // config detail: {'name': 'byDcAcCmd', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0x0c079aa78c079aa7* set_bydcaccmd(int bydcaccmd);

  // config detail: {'name': 'byDcAcWkSt', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0x0c079aa78c079aa7* set_bydcacwkst(int bydcacwkst);

  // config detail: {'name': 'byEapCmd', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0x0c079aa78c079aa7* set_byeapcmd(int byeapcmd);

  // config detail: {'name': 'byDcAc2Cmd', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0x0c079aa78c079aa7* set_bydcac2cmd(int bydcac2cmd);

  // config detail: {'name': 'byDcAc2WkSt', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0x0c079aa78c079aa7* set_bydcac2wkst(int bydcac2wkst);

 private:

  // config detail: {'name': 'byDcdcCmd', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_bydcdccmd(uint8_t* data, int bydcdccmd);

  // config detail: {'name': 'byDcAcCmd', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_bydcaccmd(uint8_t* data, int bydcaccmd);

  // config detail: {'name': 'byDcAcWkSt', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_bydcacwkst(uint8_t* data, int bydcacwkst);

  // config detail: {'name': 'byEapCmd', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_byeapcmd(uint8_t* data, int byeapcmd);

  // config detail: {'name': 'byDcAc2Cmd', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_bydcac2cmd(uint8_t* data, int bydcac2cmd);

  // config detail: {'name': 'byDcAc2WkSt', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_bydcac2wkst(uint8_t* data, int bydcac2wkst);

 private:
  int bydcdccmd_;
  int bydcaccmd_;
  int bydcacwkst_;
  int byeapcmd_;
  int bydcac2cmd_;
  int bydcac2wkst_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


