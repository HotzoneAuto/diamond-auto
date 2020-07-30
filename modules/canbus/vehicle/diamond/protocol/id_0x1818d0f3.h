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

class Id0x1818d0f3 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x1818d0f3();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

 private:

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name': 'fBatVolt', 'offset': -1000.0, 'order': 'intel', 'physical_range': '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
  double fbatvolt(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name': 'fBatCur', 'offset': 1000.0, 'order': 'intel', 'physical_range': '[1000|7553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
  double fbatcur(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name': 'fBatSOC', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|102]', 'physical_unit': '%', 'precision': 0.4, 'type': 'double'}
  double fbatsoc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name': 'byBatCellVoltHigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatcellvolthigh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 41, 'is_signed_var': False, 'len': 1, 'name': 'byBatCellVoltLow', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatcellvoltlow(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 42, 'is_signed_var': False, 'len': 1, 'name': 'byBatSocHigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatsochigh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 43, 'is_signed_var': False, 'len': 1, 'name': 'byBatSocLow', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatsoclow(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 44, 'is_signed_var': False, 'len': 1, 'name': 'byBatInnerCommErr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatinnercommerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 45, 'is_signed_var': False, 'len': 1, 'name': 'byBatVoltLow', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatvoltlow(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 46, 'is_signed_var': False, 'len': 1, 'name': 'byBatVoltHigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatvolthigh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 1, 'name': 'byBatImbalance', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatimbalance(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 1, 'name': 'byBatHeatUpRateErr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatheatuprateerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 49, 'is_signed_var': False, 'len': 1, 'name': 'byBatSmokeErr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatsmokeerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name': 'byBatInsRErr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatinsrerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 51, 'is_signed_var': False, 'len': 1, 'name': 'byBatDsgCurHigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatdsgcurhigh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 1, 'name': 'byBatChrgCurHigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatchrgcurhigh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 53, 'is_signed_var': False, 'len': 1, 'name': 'byBatTempHigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybattemphigh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 1, 'name': 'byBatTempDiffHigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybattempdiffhigh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 1, 'name': 'byBatCoolRun', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatcoolrun(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 57, 'is_signed_var': False, 'len': 1, 'name': 'byBatNegRlySts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatnegrlysts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 58, 'is_signed_var': False, 'len': 1, 'name': 'byHasLargeResCell', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byhaslargerescell(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 59, 'is_signed_var': False, 'len': 1, 'name': 'byHasSmallCapCell', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byhassmallcapcell(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 60, 'is_signed_var': False, 'len': 1, 'name': 'byHasBatChargerComm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byhasbatchargercomm(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 61, 'is_signed_var': False, 'len': 1, 'name': 'byBatChrgCC', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatchrgcc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 62, 'is_signed_var': False, 'len': 1, 'name': 'byBatInChrg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatinchrg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 1, 'name': 'byBatFuseSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatfusests(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo


