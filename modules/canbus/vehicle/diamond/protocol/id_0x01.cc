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

#include "modules/canbus/vehicle/diamond/protocol/id_0x01.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo
{
namespace canbus
{
namespace diamond
{

using ::apollo::drivers::canbus::Byte;

Id0x01::Id0x01() {}
const int32_t Id0x01::ID = 0x01;

void Id0x01::Parse(const std::uint8_t* bytes, int32_t length,
                   ChassisDetail* chassis) const
{
	chassis->mutable_diamond()->mutable_id_0x01()->set_angle_sensor_data(angle_sensor_data(bytes, length));
}

int Id0x01::angle_sensor_ID(const std::uint8_t* bytes, int32_t length)
{
	Byte t0(bytes + 1);
	int32_t x = t0.get_byte(0, 8);

	double ret = x; // 原始值：1或2，若x=1，则为前编码器；若x=2，则为后编码器
	return ret;
}

// config detail: {'name': 'angle_sensor_data', 'offset': 0.0, 'precision': 0.010986, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|360]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
/*double Id0x01::angle_sensor_data(const std::uint8_t* bytes, int32_t length, int ID_sensor = Id0x01::angle_sensor_ID(0, 8)) const
{
	// int ID_sensor = Id0x01::angle_sensor_ID(0, 8);
	if (ID_sensor == 1)
	{
		Byte t0(bytes + 4);
		int32_t x = t0.get_byte(0, 8);

		Byte t1(bytes + 3);
		int32_t t = t1.get_byte(0, 8);
		x <<= 8;
		x |= t;

		double ret_front = x * 0.010986; // 编码器当前的瞬时角度值，单位是deg
		return ret_front;
	}
	else if (ID_sensor == 2)
	{
		Byte t0(bytes + 4);
		int32_t x = t0.get_byte(0, 8);

		Byte t1(bytes + 3);
		int32_t t = t1.get_byte(0, 8);
		x <<= 8;
		x |= t;

		double ret_rear = x * 0.010986; // 编码器当前的瞬时角度值，单位是deg
		return ret_rear;
	}
}*/

double Id0x01::angle_sensor_data(const std::uint8_t* bytes, int32_t length)
{
	Byte t0(bytes + 4);
	int32_t x = t0.get_byte(0, 8);

	Byte t1(bytes + 3);
	int32_t t = t1.get_byte(0, 8);
	x <<= 8;
	x |= t;

	double ret = x * 0.010986; // 编码器当前的瞬时角度值，单位是deg
	return ret;
}// 使用时需要根据Id0x01::angle_sensor_ID函数的返回的1和2来判断是前编码器还是后编码器，来获得对应的编码器角度值

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
