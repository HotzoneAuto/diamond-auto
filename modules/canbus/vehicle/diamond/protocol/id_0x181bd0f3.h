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

class Id0x181bd0f3 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x181bd0f3();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name':
  // 'byVINRcvFlg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byvinrcvflg(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name':
  // 'byFireExtinguiErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byfireextinguierr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChrgSeatTempHigh', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bychrgseattemphigh(const std::uint8_t* bytes,
                          const int32_t length) const;

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name':
  // 'byWirelessChrgCC', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bywirelesschrgcc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name':
  // 'byMSDSts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bymsdsts(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name':
  // 'byMainRlyNotOpenErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bymainrlynotopenerr(const std::uint8_t* bytes,
                           const int32_t length) const;

  // config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatReqPowerOff', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatreqpoweroff(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 14, 'is_signed_var': False, 'len': 1, 'name':
  // 'byRemainRlyNotOpenErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byremainrlynotopenerr(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 1, 'name':
  // 'byFireAlrm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byfirealrm(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
  // 'fChrgSeatTemp1', 'offset': -40.0, 'order': 'intel', 'physical_range':
  // '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int fchrgseattemp1(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
  // 'fChrgSeatTemp2', 'offset': -40.0, 'order': 'intel', 'physical_range':
  // '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int fchrgseattemp2(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
  // 'fChrgSeatTemp3', 'offset': -40.0, 'order': 'intel', 'physical_range':
  // '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int fchrgseattemp3(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
  // 'fChrgSeatTemp4', 'offset': -40.0, 'order': 'intel', 'physical_range':
  // '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int fchrgseattemp4(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
  // 'byBatErrNum', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int bybaterrnum(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 1, 'name':
  // 'bySocJmpErr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bysocjmperr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 57, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatOverChrgErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatoverchrgerr(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 58, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatMissChrger', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatmisschrger(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 59, 'is_signed_var': False, 'len': 1, 'name':
  // 'byHVNotClose', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
  // 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byhvnotclose(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 60, 'is_signed_var': False, 'len': 1, 'name':
  // 'byBatHeatNotClose', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bybatheatnotclose(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 61, 'is_signed_var': False, 'len': 1, 'name':
  // 'byChargeCompleted', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bychargecompleted(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 62, 'is_signed_var': False, 'len': 1, 'name':
  // 'byPantographRly', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool bypantographrly(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 1, 'name':
  // 'byInsRCommErr', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  bool byinsrcommerr(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
