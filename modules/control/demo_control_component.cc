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
#include "modules/control/demo_control_component.h"

#include <string>
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "math.h"
#include "modules/control/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

namespace apollo {
namespace control {

using apollo::control::Chassis;
using apollo::control::Control_Command;
using apollo::control::Control_Reference;
using apollo::cyber::Rate;
using apollo::cyber::Time;

bool ControlComponent::Init(){
  // Reader 
  chassis_reader_ = node_->CreateReader<Chassis>(
	  FLAGS_chassis_channel, [this](const std::shared_ptr<Chassis>& chassis){
        chassis_.CopyFrom(*chassis);
      });

    // create Writer
  control_writer_ = node_->CreateWriter<Control_Command>(FLAGS_control_channel);

  // compute control message in aysnc
  async_action_ = cyber::Async(&ControlComponent::GenerateCommand, this);

  return true;
}

// write to channel
void ControlComponent::GenerateCommand() {
  auto cmd = std::make_shared<Control_Command>();
  
  // frequency, TODO: reset
  Rate rate(20.0); 

  while(true){
    float lat_dev_mgs = 5; // TODO: lateral_dev_mgs need to changed according to MGS module. 
    if(lat_dev_mgs<-4.5 || lat_dev_mgs>4.5)
	{
	  cmd->set_steering_target(10);
	}
	else
	{
	  cmd->set_steering_target(0); 
	}
	control_writer_->Write(cmd);
	rate.Sleep();
  
  }

}

ControlComponent::~ControlComponent() {
  // back chassis handle
  async_action_.wait();
}

}  // namespace control
}  // namespace apollo