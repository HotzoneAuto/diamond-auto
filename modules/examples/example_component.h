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
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/sensors/proto/sensors.pb.h"
#include "modules/sensors/proto/sensor_image.pb.h"

using apollo::cyber::Component;
using apollo::sensors::Image;
using apollo::sensors::Pose;

class ExampleComponent : public Component<Pose, Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Pose>& msg0,
            const std::shared_ptr<Image>& msg1) override;
};

CYBER_REGISTER_COMPONENT(ExampleComponent)
