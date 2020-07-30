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

class Id0x0cfff3a7 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x0cfff3a7();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'byBatRlyOffCmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0cfff3a7* set_bybatrlyoffcmd(int bybatrlyoffcmd);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'byBatRlyCmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0cfff3a7* set_bybatrlycmd(int bybatrlycmd);

 private:

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'byBatRlyOffCmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_bybatrlyoffcmd(uint8_t* data, int bybatrlyoffcmd);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'byBatRlyCmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_bybatrlycmd(uint8_t* data, int bybatrlycmd);

 private:
  int bybatrlyoffcmd_;
  int bybatrlycmd_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


