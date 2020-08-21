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

class Id0x0b19f0a8 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x0b19f0a8();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'K1_High_low_Vol_control', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'V'}
  Id0x0b19f0a8* set_k1_high_low_vol_control(int k1_high_low_vol_control);

  // config detail: {'name': 'K2_High_low_Vol_control', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': 'V'}
  Id0x0b19f0a8* set_k2_high_low_vol_control(int k2_high_low_vol_control);

 private:

  // config detail: {'name': 'K1_High_low_Vol_control', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'V'}
  void set_p_k1_high_low_vol_control(uint8_t* data, int k1_high_low_vol_control);

  // config detail: {'name': 'K2_High_low_Vol_control', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|10]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': 'V'}
  void set_p_k2_high_low_vol_control(uint8_t* data, int k2_high_low_vol_control);

 private:
  int k1_high_low_vol_control_;
  int k2_high_low_vol_control_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


