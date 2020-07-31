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

#include "modules/canbus/vehicle/diamond/diamond_vehicle_factory.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/diamond/diamond_controller.h"
#include "modules/canbus/vehicle/diamond/diamond_message_manager.h"
#include "modules/common/util/util.h"

namespace apollo {
namespace canbus {

std::unique_ptr<VehicleController>
DiamondVehicleFactory::CreateVehicleController() {
  return std::unique_ptr<VehicleController>(new diamond::DiamondController());
}

std::unique_ptr<MessageManager<::apollo::canbus::ChassisDetail>>
DiamondVehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager<::apollo::canbus::ChassisDetail>>(
      new diamond::DiamondMessageManager());
}

}  // namespace canbus
}  // namespace apollo
