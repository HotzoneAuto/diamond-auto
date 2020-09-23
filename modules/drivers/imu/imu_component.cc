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
#include "modules/drivers/imu/imu_component.h"

#include <memory>
#include <string>
#include <thread>

#include "modules/common/util/message_util.h"

namespace apollo {
namespace drivers {
namespace imu {

bool LPMSDriverComponent::Init() {
  if (!GetProtoConfig(&device_conf_)) {
    return false;
  }
  AINFO << "LPMS config: " << device_conf_.DebugString();

  // enable resonable log output for OpenZen
  ZenSetLogLevel(ZenLogLevel_Info);

  // create OpenZen Client
  auto clientPair = make_client();
  auto& clientError = clientPair.first;
  client_ = clientPair.second;

  if (clientError) {
    AERROR << "Cannot create OpenZen client" << clientError;
    return false;
  }

  // connect to sensor on IO System by the sensor name
  auto sensorPair =
      client_.get().obtainSensorByName("dev", device_conf_.device_id());
  auto& obtainError = sensorPair.first;
  auto& sensor = sensorPair.second;
  if (obtainError) {
    client_.get().close();
    AERROR << "Cannot connect to sensor" << obtainError;
    return false;
  }

  // check that the sensor has an IMU component
  auto imuPair = sensor.getAnyComponentOfType(g_zenSensorType_Imu);
  auto& hasImu = imuPair.first;
  imu_ = imuPair.second;

  if (!hasImu) {
    client_.get().close();
    AERROR << "Connected sensor has no IMU" << ZenError_WrongSensorType;
    return false;
  }

  return true;
}

bool LPMSDriverComponent::Action() {
  auto event = client_.get().waitForNextEvent();
  if (event.second.component.handle == imu_.component().handle) {
    std::cout << "> Acceleration: \t x = " << event.second.data.imuData.a[0]
              << "\t y = " << event.second.data.imuData.a[1]
              << "\t z = " << event.second.data.imuData.a[2] << std::endl;
    std::cout << "> Gyro: \t\t x = " << event.second.data.imuData.g[0]
              << "\t y = " << event.second.data.imuData.g[1]
              << "\t z = " << event.second.data.imuData.g[2] << std::endl;

    auto const& d = event.second.data.imuData;

    apollo::drivers::Imu imu_pub;

    auto header = imu_pub.mutable_header();
    header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
    header->set_frame_id("imu");

    // Fill orientation quaternion
    auto orientation = imu_pub.mutable_orientation();
    orientation->set_qw(d.q[0]);
    orientation->set_qx(-d.q[1]);
    orientation->set_qy(-d.q[2]);
    orientation->set_qz(-d.q[3]);

    // Fill angular velocity data
    //   // - scale from deg/s to rad/s
    auto angular_velocity = imu_pub.mutable_angular_velocity();
    angular_velocity->set_x(d.g[0] * cDegToRad);
    angular_velocity->set_y(d.g[1] * cDegToRad);
    angular_velocity->set_z(d.g[2] * cDegToRad);

    // Fill linear acceleration data
    const float rosConversion = -1.0 * (!false) + 1.0 * false;

    auto linear_acceleration = imu_pub.mutable_linear_acceleration();
    linear_acceleration->set_x(rosConversion * d.a[0] * cEarthG);
    linear_acceleration->set_y(rosConversion * d.a[1] * cEarthG);
    linear_acceleration->set_z(rosConversion * d.a[2] * cEarthG);

    // Units are microTesla in the LPMS library, Tesla in ROS.
    auto magnetic_field = imu_pub.mutable_magnetic_field();
    magnetic_field->set_x(d.b[0] * cMicroToTelsa);
    magnetic_field->set_y(d.b[1] * cMicroToTelsa);
    magnetic_field->set_z(d.b[2] * cMicroToTelsa);

    // Publish the messages
    imu_writer_->Write(imu_pub);
  }

  return true;
}

}  // namespace imu
}  // namespace drivers
}  // namespace apollo
