/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <thread>

#include "OpenZen.h"

#include "cyber/component/component.h"
#include "cyber/cyber.h"

#include "modules/drivers/imu/proto/device_config.pb.h"
#include "modules/drivers/proto/imu.pb.h"

namespace apollo {
namespace drivers {
namespace imu {

using namespace zen;
using apollo::cyber::Component;
using apollo::cyber::Writer;

class LPMSDriverComponent : public Component<> {
 public:
  LPMSDriverComponent();
  ~LPMSDriverComponent();

  bool Init() override;
  bool Action();

 private:
  apollo::drivers::imu::IMUDeviceConf device_conf_;

  std::reference_wrapper<ZenClient> client_;
  ZenSensorComponent imu_;

  const float cDegToRad = 3.1415926f / 180.0f;
  const float cEarthG = 9.81f;
  const float cMicroToTelsa = 1e-6f;

  std::shared_ptr<apollo::cyber::Writer<Imu>> imu_writer_;
};

CYBER_REGISTER_COMPONENT(LPMSDriverComponent)

}  // namespace imu
}  // namespace drivers
}  // namespace apollo
