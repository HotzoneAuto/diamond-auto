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

class Id0x0c089aa78c089aa7 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x0c089aa78c089aa7();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'fDcAcTargSpd', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'rpm'}
  Id0x0c089aa78c089aa7* set_fdcactargspd(int fdcactargspd);

  // config detail: {'name': 'byDcAcDir', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0x0c089aa78c089aa7* set_bydcacdir(int bydcacdir);

  // config detail: {'name': 'fDcAc2TargSpd', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': 'rpm'}
  Id0x0c089aa78c089aa7* set_fdcac2targspd(int fdcac2targspd);

  // config detail: {'name': 'byDcAc2Dir', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0x0c089aa78c089aa7* set_bydcac2dir(int bydcac2dir);

 private:

  // config detail: {'name': 'fDcAcTargSpd', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'rpm'}
  void set_p_fdcactargspd(uint8_t* data, int fdcactargspd);

  // config detail: {'name': 'byDcAcDir', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_bydcacdir(uint8_t* data, int bydcacdir);

  // config detail: {'name': 'fDcAc2TargSpd', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': 'rpm'}
  void set_p_fdcac2targspd(uint8_t* data, int fdcac2targspd);

  // config detail: {'name': 'byDcAc2Dir', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_bydcac2dir(uint8_t* data, int bydcac2dir);

 private:
  int fdcactargspd_;
  int bydcacdir_;
  int fdcac2targspd_;
  int bydcac2dir_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


