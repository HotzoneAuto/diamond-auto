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

#include "modules/canbus/vehicle/diamond/protocol/id_0x1818d0f3.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x1818d0f3::Id0x1818d0f3() {}
const int32_t Id0x1818d0f3::ID = 0x1818D0F3;

void Id0x1818d0f3::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_fbatvolt(
      fbatvolt(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_fbatcur(
      fbatcur(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_fbatsoc(
      fbatsoc(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatcellvolthigh(
      bybatcellvolthigh(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatcellvoltlow(
      bybatcellvoltlow(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatsochigh(
      bybatsochigh(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatsoclow(
      bybatsoclow(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatinnercommerr(
      bybatinnercommerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatvoltlow(
      bybatvoltlow(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatvolthigh(
      bybatvolthigh(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatimbalance(
      bybatimbalance(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatheatuprateerr(
      bybatheatuprateerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatsmokeerr(
      bybatsmokeerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatinsrerr(
      bybatinsrerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatdsgcurhigh(
      bybatdsgcurhigh(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatchrgcurhigh(
      bybatchrgcurhigh(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybattemphigh(
      bybattemphigh(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybattempdiffhigh(
      bybattempdiffhigh(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatcoolrun(
      bybatcoolrun(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatnegrlysts(
      bybatnegrlysts(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_byhaslargerescell(
      byhaslargerescell(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_byhassmallcapcell(
      byhassmallcapcell(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_byhasbatchargercomm(
      byhasbatchargercomm(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatchrgcc(
      bybatchrgcc(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatinchrg(
      bybatinchrg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x1818d0f3()->set_bybatfusests(
      bybatfusests(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 16, 'name':
// 'fbatvolt', 'offset': -1000.0, 'order': 'intel', 'physical_range':
// '[-1000|5553.5]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Id0x1818d0f3::fbatvolt(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -1000.000000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 16, 'name':
// 'fbatcur', 'offset': 1000.0, 'order': 'intel', 'physical_range':
// '[1000|7553.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Id0x1818d0f3::fbatcur(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + 1000.000000;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
// 'fbatsoc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|102]',
// 'physical_unit': '%', 'precision': 0.4, 'type': 'double'}
double Id0x1818d0f3::fbatsoc(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 0.400000;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatcellvolthigh', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatcellvolthigh(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 41, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatcellvoltlow', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatcellvoltlow(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 42, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatsochigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatsochigh(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 43, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatsoclow', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatsoclow(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatinnercommerr', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatinnercommerr(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 45, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatvoltlow', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatvoltlow(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 46, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatvolthigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatvolthigh(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatimbalance', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatimbalance(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatheatuprateerr', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatheatuprateerr(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 49, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatsmokeerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatsmokeerr(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatinsrerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatinsrerr(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 51, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatdsgcurhigh', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatdsgcurhigh(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatchrgcurhigh', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatchrgcurhigh(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 53, 'is_signed_var': False, 'len': 1, 'name':
// 'bybattemphigh', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybattemphigh(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 55, 'is_signed_var': False, 'len': 1, 'name':
// 'bybattempdiffhigh', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybattempdiffhigh(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatcoolrun', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatcoolrun(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 57, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatnegrlysts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatnegrlysts(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 58, 'is_signed_var': False, 'len': 1, 'name':
// 'byhaslargerescell', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::byhaslargerescell(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 59, 'is_signed_var': False, 'len': 1, 'name':
// 'byhassmallcapcell', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::byhassmallcapcell(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 60, 'is_signed_var': False, 'len': 1, 'name':
// 'byhasbatchargercomm', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::byhasbatchargercomm(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 61, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatchrgcc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatchrgcc(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 62, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatinchrg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatinchrg(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 1, 'name':
// 'bybatfusests', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x1818d0f3::bybatfusests(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
