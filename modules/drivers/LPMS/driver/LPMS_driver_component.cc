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

#include <memory>
#include <string>
#include <thread>

#include "modules/common/util/message_util.h"
#include "modules/drivers/LPMS/driver/LPMS_driver_component.h"

namespace apollo {
namespace drivers {
namespace LPMS {

bool LPMSDriverComponent::Init() {
  AINFO << "LPMS driver component init";
  Config LPMS_config;
  if (!GetProtoConfig(&LPMS_config)) {
    return false;
  }
  AINFO << "LPMS config: " << LPMS_config.DebugString();

  if (driver == nullptr) {
    return false;
  }
  dvr_.reset(driver);
  dvr_->Init();
  // spawn device poll thread
  runing_ = true;
  /*
  other essential contents in initializing function
  */

  device_thread_->detach();
  return true;
}


bool LPMSDriverComponent::Proc(const std::shared_ptr<Driver>& msg){
  if (!CheckInput()) {
    AERROR << "Input check failed!";
    return false;
  }//check whether the input message is right

  /*
  other essential contents in proceeding function
  */

  /*
  receive and publish message from IMU
  LPMS_writer_->Write(std::...);
  */
  return true;
}

/*
other essential contents
*/
}  // namespace LPMS
}  // namespace drivers
}  // namespace apollo
