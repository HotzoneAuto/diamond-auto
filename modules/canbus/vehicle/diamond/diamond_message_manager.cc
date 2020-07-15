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

#include "modules/canbus/vehicle/diamond/diamond_message_manager.h"

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c079aa7_8c079aa7.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c089aa7_8c089aa7.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c19f0a7_8c19f0a7.h"

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c08a7f0_8c08a7f0.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c09a7f0_8c09a7f0.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c0ba7f0_8c0ba7f0.h"

namespace apollo {
namespace canbus {
namespace diamond {

DiamondMessageManager::DiamondMessageManager() {
  // Control Messages
  AddSendProtocolData<Id0x0c079aa78c079aa7, true>();
  AddSendProtocolData<Id0x0c089aa78c089aa7, true>();
  AddSendProtocolData<Id0x0c19f0a78c19f0a7, true>();

  // Report Messages
  AddRecvProtocolData<Id0x0c08a7f08c08a7f0, true>();
  AddRecvProtocolData<Id0x0c09a7f08c09a7f0, true>();
  AddRecvProtocolData<Id0x0c0ba7f08c0ba7f0, true>();
}

DiamondMessageManager::~DiamondMessageManager() {}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
