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

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'fMotMaxPower', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': 'kw', 'precision': 1.0, 'type': 'int'}
  Id0x0c0ba7f08c0ba7f0* set_fmotmaxpower(int fmotmaxpower);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'fMotMaxSpd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25500]', 'physical_unit': 'rpm', 'precision': 100.0, 'type': 'double'}
  Id0x0c0ba7f08c0ba7f0* set_fmotmaxspd(double fmotmaxspd);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'fMotMaxTq', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
  Id0x0c0ba7f08c0ba7f0* set_fmotmaxtq(int fmotmaxtq);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 35, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 37, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 38, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 41, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 42, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 43, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(bool dwmcuerrflg);

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 20, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1048575]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Id0x0c0ba7f08c0ba7f0* set_dwmcuerrflg(int dwmcuerrflg);

 private:

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'fMotMaxPower', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': 'kw', 'precision': 1.0, 'type': 'int'}
  void set_p_fmotmaxpower(uint8_t* data, int fmotmaxpower);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'fMotMaxSpd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|25500]', 'physical_unit': 'rpm', 'precision': 100.0, 'type': 'double'}
  void set_p_fmotmaxspd(uint8_t* data, double fmotmaxspd);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'fMotMaxTq', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|65535]', 'physical_unit': 'NM', 'precision': 1.0, 'type': 'int'}
  void set_p_fmotmaxtq(uint8_t* data, int fmotmaxtq);

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 35, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 37, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 38, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 41, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 42, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 43, 'is_signed_var': False, 'len': 1, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_dwmcuerrflg(uint8_t* data, bool dwmcuerrflg);

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 20, 'name': 'dwMCUErrFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1048575]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_dwmcuerrflg(uint8_t* data, int dwmcuerrflg);

 private:
  int fmotmaxpower_;
  double fmotmaxspd_;
  int fmotmaxtq_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  bool dwmcuerrflg_;
  int dwmcuerrflg_;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


