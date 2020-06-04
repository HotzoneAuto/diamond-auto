/******************************************************************************
 * MIT License

 * Copyright (c) 2019 Geekstyle

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
******************************************************************************/
#pragma once

#include <map>
#include <memory>
#include <string>

#include "librealsense2/rs.hpp"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/node/node.h"

#include "modules/common/global_gflags.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/drivers/proto/drivers.pb.h"
#include "modules/drivers/realsense/device_base.h"
#include "modules/drivers/realsense/realsense_d435.h"
#include "modules/drivers/realsense/realsense_d435i.h"
#include "modules/drivers/realsense/realsense_t265.h"

namespace apollo {
namespace drivers {

using apollo::control::Chassis;
using apollo::cyber::Component;
using apollo::drivers::PointCloud;
using apollo::drivers::realsense::D435;
using apollo::drivers::realsense::D435I;
using apollo::drivers::realsense::DeviceBase;
using apollo::drivers::realsense::T265;

class RealsenseComponent : public Component<> {
 public:
  bool Init() override;
  void InitDeviceAndSensor();
  void Run();
  ~RealsenseComponent();

 private:
  // realsense device
  rs2::device device_;

  // device object like T265\D435I\D435
  DeviceBase *device_object_;
};

CYBER_REGISTER_COMPONENT(RealsenseComponent)
}  // namespace drivers
}  // namespace apollo
