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
#include "ManagedThread.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"

#include "modules/drivers/LPMS/proto/service.pb.h"
#include "modules/drivers/proto/imu.pb.h"


namespace apollo {
namespace drivers {
namespace LPMS {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::LPMS::Content;
using apollo::drivers::LPMS::Request;

class LPMSDriverComponent : public Component<>
{
public:
	std::shared_ptr<apollo::cyber::Node> node_;

	// Publisher
	std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Imu>> imu_writer_ =
		nullptr;

	// Service

	// Parameters
	std::string m_sensorName;
	std::string m_sensorInterface;
	std::string frame_id;
	int m_baudrate = 0;
	  
	LPMSDriverComponent() = default
	~LPMSDriverComponent() 
	{
		if (device_thread_->joinable()) 
		{
			device_thread_->join();
		}
	}
	bool Init() override;
	// bool Proc(const std::shared_ptr<Driver>& msg) override;
	  
	bool run(void);
	void publishIsAutocalibrationActive();
	bool setAutocalibration(const std::shared_ptr<Request>& req, 
							  const std::shared_ptr<Content>& res);
	bool resetHeading(const std::shared_ptr<Content>& request,
						std::shared_ptr<Content>& response);
	bool calibrateGyroscope(const std::shared_ptr<Content>& req,
							  const std::shared_ptr<Content>& res);

private:
	std::unique_ptr<zen::ZenClient> m_zenClient;
	std::unique_ptr<zen::ZenSensor> m_zenSensor;
	std::unique_ptr<zen::ZenSensorComponent> m_zenImu;//receive and send message
	  
	bool m_openzenVerbose;
	bool m_useLpmsAccelerationConvention;

	struct SensorThreadParams {
		zen::ZenClient* zenClient;
		std::string frame_id;
		std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Imu>> imu_writer_;
		bool useLpmsAccelerationConvention;
	};

	ManagedThread<SensorThreadParams> m_sensorThread;
	  
	/*
	const float cDegToRad = 3.1415926f / 180.0f;
	const float cEarthG = 9.81f;
	const float cMicroToTelsa = 1e-6f;
	*/

};

CYBER_REGISTER_COMPONENT(LPMSDriverComponent)

}  // namespace LPMS
}  // namespace drivers
}  // namespace apollo
