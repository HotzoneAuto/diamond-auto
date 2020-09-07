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

#include "modules/canbus/vehicle/diamond/protocol/id_0x00aa5701.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x01.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x03.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x04.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c079aa7.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c19f0a7.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0cfff3a7.h"

#include "modules/canbus/vehicle/diamond/protocol/id_0x0c08a7f0.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c09a7f0.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x0c0ba7f0.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x1818d0f3.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x1819d0f3.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x181ad0f3.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x181bd0f3.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x181cd0f3.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x181dd0f3.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x181ed0f3.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x181fd0f3.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x1825d0f3.h"
#include "modules/canbus/vehicle/diamond/protocol/id_0x18eba1a5.h"

namespace apollo {
namespace canbus {
namespace diamond {

DiamondMessageManager::DiamondMessageManager() {
  // Control Messages
  AddSendProtocolData<Id0x0c079aa7, true>();
  AddSendProtocolData<Id0x0c19f0a7, true>();
  AddSendProtocolData<Id0x0cfff3a7, true>();
  AddSendProtocolData<Id0x00aa5701, true>();

  // Report Messages
  AddRecvProtocolData<Id0x01, true>();
  AddRecvProtocolData<Id0x03, true>();
  AddRecvProtocolData<Id0x04, true>();
  AddRecvProtocolData<Id0x0c08a7f0, true>();
  AddRecvProtocolData<Id0x0c09a7f0, true>();
  AddRecvProtocolData<Id0x0c0ba7f0, true>();
  AddRecvProtocolData<Id0x1818d0f3, true>();
  AddRecvProtocolData<Id0x1819d0f3, true>();
  AddRecvProtocolData<Id0x181ad0f3, true>();
  AddRecvProtocolData<Id0x181bd0f3, true>();
  AddRecvProtocolData<Id0x181cd0f3, true>();
  AddRecvProtocolData<Id0x181dd0f3, true>();
  AddRecvProtocolData<Id0x181ed0f3, true>();
  AddRecvProtocolData<Id0x181fd0f3, true>();
  AddRecvProtocolData<Id0x1825d0f3, true>();
  AddRecvProtocolData<Id0x18eba1a5, true>();
}

DiamondMessageManager::~DiamondMessageManager() {}

}  // namespace diamond
}  // namespace canbus
}  // namespace apollo
