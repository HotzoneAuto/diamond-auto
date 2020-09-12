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

#include "modules/canbus/vehicle/diamond/protocol/id_0x181bd0f3.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x181bd0f3::Id0x181bd0f3() {}
const int32_t Id0x181bd0f3::ID = 0x181BD0F3;

void Id0x181bd0f3::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  auto id_181bd0f3 = chassis->mutable_diamond()->mutable_id_0x181bd0f3();
  id_181bd0f3->set_byvinrcvflg(byvinrcvflg(bytes, length));
  id_181bd0f3->set_byfireextinguierr(byfireextinguierr(bytes, length));
  id_181bd0f3->set_bychrgseattemphigh(bychrgseattemphigh(bytes, length));
  id_181bd0f3->set_bywirelesschrgcc(bywirelesschrgcc(bytes, length));
  id_181bd0f3->set_bymsdsts(bymsdsts(bytes, length));
  id_181bd0f3->set_bymainrlynotopenerr(bymainrlynotopenerr(bytes, length));
  id_181bd0f3->set_bybatreqpoweroff(bybatreqpoweroff(bytes, length));
  id_181bd0f3->set_byremainrlynotopenerr(byremainrlynotopenerr(bytes, length));
  id_181bd0f3->set_byfirealrm(byfirealrm(bytes, length));
  id_181bd0f3->set_fchrgseattemp1(fchrgseattemp1(bytes, length));
  id_181bd0f3->set_fchrgseattemp2(fchrgseattemp2(bytes, length));
  id_181bd0f3->set_fchrgseattemp3(fchrgseattemp3(bytes, length));
  id_181bd0f3->set_fchrgseattemp4(fchrgseattemp4(bytes, length));
  id_181bd0f3->set_bybaterrnum(bybaterrnum(bytes, length));
  id_181bd0f3->set_bysocjmperr(bysocjmperr(bytes, length));
  id_181bd0f3->set_bybatoverchrgerr(bybatoverchrgerr(bytes, length));
  id_181bd0f3->set_bybatmisschrger(bybatmisschrger(bytes, length));
  id_181bd0f3->set_byhvnotclose(byhvnotclose(bytes, length));
  id_181bd0f3->set_bybatheatnotclose(bybatheatnotclose(bytes, length));
  id_181bd0f3->set_bychargecompleted(bychargecompleted(bytes, length));
  id_181bd0f3->set_bypantographrly(bypantographrly(bytes, length));
  id_181bd0f3->set_byinsrcommerr(byinsrcommerr(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name':
// 'byvinrcvflg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::byvinrcvflg(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name':
// 'byfireextinguierr', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::byfireextinguierr(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name':
// 'bychrgseattemphigh', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bychrgseattemphigh(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 10, 'is_signed_var': False, 'len': 1, 'name':
// 'bywirelesschrgcc', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bywirelesschrgcc(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 11, 'is_signed_var': False, 'len': 1, 'name':
// 'bymsdsts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bymsdsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name':
// 'bymainrlynotopenerr', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bymainrlynotopenerr(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatreqpoweroff', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bybatreqpoweroff(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 14, 'is_signed_var': False, 'len': 1, 'name':
// 'byremainrlynotopenerr', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::byremainrlynotopenerr(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 1, 'name':
// 'byfirealrm', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::byfirealrm(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
// 'fchrgseattemp1', 'offset': -40.0, 'order': 'intel', 'physical_range':
// '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x181bd0f3::fchrgseattemp1(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
// 'fchrgseattemp2', 'offset': -40.0, 'order': 'intel', 'physical_range':
// '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x181bd0f3::fchrgseattemp2(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
// 'fchrgseattemp3', 'offset': -40.0, 'order': 'intel', 'physical_range':
// '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x181bd0f3::fchrgseattemp3(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
// 'fchrgseattemp4', 'offset': -40.0, 'order': 'intel', 'physical_range':
// '[-40|215]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x181bd0f3::fchrgseattemp4(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x + -40.000000;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 8, 'name':
// 'bybaterrnum', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x181bd0f3::bybaterrnum(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 1, 'name':
// 'bysocjmperr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bysocjmperr(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 57, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatoverchrgerr', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bybatoverchrgerr(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 58, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatmisschrger', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bybatmisschrger(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 59, 'is_signed_var': False, 'len': 1, 'name':
// 'byhvnotclose', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::byhvnotclose(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 60, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatheatnotclose', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bybatheatnotclose(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 61, 'is_signed_var': False, 'len': 1, 'name':
// 'bychargecompleted', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bychargecompleted(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 62, 'is_signed_var': False, 'len': 1, 'name':
// 'bypantographrly', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::bypantographrly(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 1, 'name':
// 'byinsrcommerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181bd0f3::byinsrcommerr(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
