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

#include "modules/canbus/vehicle/diamond/protocol/id_0x181ed0f3.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace diamond {

using ::apollo::drivers::canbus::Byte;

Id0x181ed0f3::Id0x181ed0f3() {}
const int32_t Id0x181ed0f3::ID = 0x181ED0F3;

void Id0x181ed0f3::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_byfuelbatclosecmd(byfuelbatclosecmd(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_byparkchrgdisable(byparkchrgdisable(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_byvehtmsdcdcsts(byvehtmsdcdcsts(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_byvehtmsdcdcerr(byvehtmsdcdcerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_byvehtmserr(byvehtmserr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_byinsrjump(byinsrjump(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybatdcdcerr(bybatdcdcerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybattemplow(bybattemplow(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybatheatcircuiterr(bybatheatcircuiterr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybatselfprotecterr(bybatselfprotecterr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bycanacommerr(bycanacommerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybatbranchbreakerr(bybatbranchbreakerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybatinmsderr(bybatinmsderr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bytmsrlynotcloseerr(bytmsrlynotcloseerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bytmsrlynotopenerr(bytmsrlynotopenerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybatnegrlynotcloseerr(bybatnegrlynotcloseerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bychrgaposrlynotopenerr(bychrgaposrlynotopenerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bychrgaposrlynotcloseerr(bychrgaposrlynotcloseerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bychrganegrlynotcloseerr(bychrganegrlynotcloseerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bychrganegrlynotopenerr(bychrganegrlynotopenerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bychrgbposrlynotcloseerr(bychrgbposrlynotcloseerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bychrgbposrlynotopenerr(bychrgbposrlynotopenerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bychrgbnegrlynotcloseerr(bychrgbnegrlynotcloseerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bychrgbnegrlynotopenerr(bychrgbnegrlynotopenerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bydsgcurhghwhenchrg(bydsgcurhghwhenchrg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bychrgseatntcerr(bychrgseatntcerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybatoverdsg(bybatoverdsg(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybatcursensorerr(bybatcursensorerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybmslowvolterr(bybmslowvolterr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybatchrgsignalerr(bybatchrgsignalerr(bytes, length));
  chassis->mutable_diamond()->mutable_id_0x181ed0f3()->set_bybmsmode(bybmsmode(bytes, length));
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 1, 'name': 'byfuelbatclosecmd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::byfuelbatclosecmd(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 33, 'is_signed_var': False, 'len': 1, 'name': 'byparkchrgdisable', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::byparkchrgdisable(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 34, 'is_signed_var': False, 'len': 1, 'name': 'byvehtmsdcdcsts', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::byvehtmsdcdcsts(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 35, 'is_signed_var': False, 'len': 1, 'name': 'byvehtmsdcdcerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::byvehtmsdcdcerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 36, 'is_signed_var': False, 'len': 1, 'name': 'byvehtmserr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::byvehtmserr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 37, 'is_signed_var': False, 'len': 1, 'name': 'byinsrjump', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::byinsrjump(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 38, 'is_signed_var': False, 'len': 1, 'name': 'bybatdcdcerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybatdcdcerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 1, 'name': 'bybattemplow', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybattemplow(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name': 'bybatheatcircuiterr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybatheatcircuiterr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 41, 'is_signed_var': False, 'len': 1, 'name': 'bybatselfprotecterr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybatselfprotecterr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 42, 'is_signed_var': False, 'len': 1, 'name': 'bycanacommerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bycanacommerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 43, 'is_signed_var': False, 'len': 1, 'name': 'bybatbranchbreakerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybatbranchbreakerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 44, 'is_signed_var': False, 'len': 1, 'name': 'bybatinmsderr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybatinmsderr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 45, 'is_signed_var': False, 'len': 1, 'name': 'bytmsrlynotcloseerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bytmsrlynotcloseerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 46, 'is_signed_var': False, 'len': 1, 'name': 'bytmsrlynotopenerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bytmsrlynotopenerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 1, 'name': 'bybatnegrlynotcloseerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybatnegrlynotcloseerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 48, 'is_signed_var': False, 'len': 1, 'name': 'bychrgaposrlynotopenerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bychrgaposrlynotopenerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 49, 'is_signed_var': False, 'len': 1, 'name': 'bychrgaposrlynotcloseerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bychrgaposrlynotcloseerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 50, 'is_signed_var': False, 'len': 1, 'name': 'bychrganegrlynotcloseerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bychrganegrlynotcloseerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 51, 'is_signed_var': False, 'len': 1, 'name': 'bychrganegrlynotopenerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bychrganegrlynotopenerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 52, 'is_signed_var': False, 'len': 1, 'name': 'bychrgbposrlynotcloseerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bychrgbposrlynotcloseerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 53, 'is_signed_var': False, 'len': 1, 'name': 'bychrgbposrlynotopenerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bychrgbposrlynotopenerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 54, 'is_signed_var': False, 'len': 1, 'name': 'bychrgbnegrlynotcloseerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bychrgbnegrlynotcloseerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 55, 'is_signed_var': False, 'len': 1, 'name': 'bychrgbnegrlynotopenerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bychrgbnegrlynotopenerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(7, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 56, 'is_signed_var': False, 'len': 1, 'name': 'bydsgcurhghwhenchrg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bydsgcurhghwhenchrg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 57, 'is_signed_var': False, 'len': 1, 'name': 'bychrgseatntcerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bychrgseatntcerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 58, 'is_signed_var': False, 'len': 1, 'name': 'bybatoverdsg', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybatoverdsg(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 59, 'is_signed_var': False, 'len': 1, 'name': 'bybatcursensorerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybatcursensorerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 60, 'is_signed_var': False, 'len': 1, 'name': 'bybmslowvolterr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybmslowvolterr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 61, 'is_signed_var': False, 'len': 1, 'name': 'bybatchrgsignalerr', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
bool Id0x181ed0f3::bybatchrgsignalerr(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'bit': 62, 'is_signed_var': False, 'len': 2, 'name': 'bybmsmode', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|3]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Id0x181ed0f3::bybmsmode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}
}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
