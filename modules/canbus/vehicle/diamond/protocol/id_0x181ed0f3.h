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

class Id0x181ed0f3 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x181ed0f3();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name':
  // 'byFuelBatCloseCmd', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byfuelbatclosecmd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name':
  // 'byParkChrgDisable', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byparkchrgdisable(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name':
  // 'byVehTMSDcdcSts', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byvehtmsdcdcsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 35, 'is_signed_var': False, 'len': 1, 'name':
  // 'byVehTMSDcdcErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byvehtmsdcdcerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 36, 'is_signed_var': False, 'len': 1, 'name':
  // 'byVehTMSErr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byvehtmserr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 37, 'is_signed_var': False, 'len': 1, 'name':
  // 'byInsRJump', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byinsrjump(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 38, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatDcdcErr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatdcdcerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 39, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatTempLow', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybattemplow(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatHeatCircuitErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatheatcircuiterr(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 41, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatSelfProtectErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatselfprotecterr(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 42, 'is_signed_var': False, 'len': 1, 'name':
  // 'byCANACommErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bycanacommerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 43, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatBranchBreakErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatbranchbreakerr(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatInMsdErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatinmsderr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 45, 'is_signed_var': False, 'len': 1, 'name':
  // 'byTMSRlyNotCloseErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bytmsrlynotcloseerr(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 46, 'is_signed_var': False, 'len': 1, 'name':
  // 'byTMSRlyNotOpenErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bytmsrlynotopenerr(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatNegRlyNotCloseErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool bybatnegrlynotcloseerr(const std::uint8_t* bytes,
                              const int32_t length) const;

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgAPosRlyNotOpenErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool bychrgaposrlynotopenerr(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'bit': 49, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgAPosRlyNotCloseErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool bychrgaposrlynotcloseerr(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgANegRlyNotCloseErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool bychrganegrlynotcloseerr(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'bit': 51, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgANegRlyNotOpenErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool bychrganegrlynotopenerr(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgBPosRlyNotCloseErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool bychrgbposrlynotcloseerr(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'bit': 53, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgBPosRlyNotOpenErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool bychrgbposrlynotopenerr(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'bit': 54, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgBNegRlyNotCloseErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool bychrgbnegrlynotcloseerr(const std::uint8_t* bytes,
                                const int32_t length) const;

  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgBNegRlyNotOpenErr', 'offset': 0.0, 'order': 'intel',
  // 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type':
  // 'bool'}
  bool bychrgbnegrlynotopenerr(const std::uint8_t* bytes,
                               const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 1, 'name':
  // 'byDsgCurHghWhenChrg', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bydsgcurhghwhenchrg(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 57, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgSeatNTCErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bychrgseatntcerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 58, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatOverDsg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatoverdsg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 59, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatCurSensorErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatcursensorerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 60, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBmsLowVoltErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybmslowvolterr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 61, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatChrgSignalErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatchrgsignalerr(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 62, 'is_signed_var': False, 'len': 2, 'name':
  // 'byBMSMode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bybmsmode(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
