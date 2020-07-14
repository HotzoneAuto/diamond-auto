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

class Id0x0c0ba7f08c0ba7f0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x0c0ba7f08c0ba7f0();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'name': 'fMotMaxPower', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': 'kw'}
  int fmotmaxpower(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'fMotMaxSpd', 'offset': 0.0, 'precision': 100.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|25500]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': 'rpm'}
  double fmotmaxspd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'fMotMaxTq', 'offset': 0.0, 'precision': 1.0, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 16, 'type': 'int', 'order': 'intel', 'physical_unit': 'NM'}
  int fmotmaxtq(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 32, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 33, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 34, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 35, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 36, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 37, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 38, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 39, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 40, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 41, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 42, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 43, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  bool dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'dwMCUErrFlg', 'offset': 0.0, 'precision': 1.0, 'len': 20, 'is_signed_var': False, 'physical_range': '[0|1048575]', 'bit': 44, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int dwmcuerrflg(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


