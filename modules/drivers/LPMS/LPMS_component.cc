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
#include "modules/drivers/LPMS/LPMS_component.h"


namespace apollo
{
namespace drivers
{
namespace LPMS
{

bool LPMSDriverComponent::m_sensorThread(LPMSDriverComponent::SensorThreadParams const& param) // read data from IMU
{
	const float cDegToRad = 3.1415926f / 180.0f;
	const float cEarthG = 9.81f;
	const float cMicroToTelsa = 1e-6f;

	auto event = param.zenClient->waitForNextEvent();
	auto have_event = event.first;
	auto event_value = event.second;

	if (!have_event)
	{
		// empty event received, terminate
		return false;
	}

	if (!event_value.component.handle)
	{
		// not an event from a component
		switch (event_value.eventType)
		{
		case ZenSensorEvent_SensorDisconnected:
			AINFO << "OpenZen sensor disconnected";
			return false;
		}
	}

	if (event_value.component.handle == 1)
	{
		AINFO << "event_value.component.handle == 1";
		if (event_value.eventType == ZenImuEvent_Sample)
		{
			// IMU
			auto const& d = event_value.data.imuData;

			apollo::drivers::Imu imu;

			auto header = imu.mutable_header();
			header->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
			header->set_frame_id("imu");

			//   Fill orientation quaternion
			auto orientation = imu.mutable_orientation();
			orientation->set_qw(d.q[0]);
			orientation->set_qx(-d.q[1]);
			orientation->set_qy(-d.q[2]);
			orientation->set_qz(-d.q[3]);

			//   Fill angular velocity data
			//   scale from deg/s to rad/s
			auto angular_velocity = imu.mutable_angular_velocity();
			angular_velocity->set_x(d.g[0] * cDegToRad);
			angular_velocity->set_y(d.g[1] * cDegToRad);
			angular_velocity->set_z(d.g[2] * cDegToRad);

			//   Fill linear acceleration data
			const float rosConversion =
			    -1.0 * (!param.useLpmsAccelerationConvention) +
			    1.0 * param.useLpmsAccelerationConvention;

			auto linear_acceleration = imu.mutable_linear_acceleration();
			linear_acceleration->set_x(rosConversion * d.a[0] * cEarthG);
			linear_acceleration->set_y(rosConversion * d.a[1] * cEarthG);
			linear_acceleration->set_z(rosConversion * d.a[2] * cEarthG);

			// Units are microTesla in the LPMS library, Tesla in ROS.
			auto magnetic_field = imu.mutable_magnetic_field();
			magnetic_field->set_x(d.b[0] * cMicroToTelsa);
			magnetic_field->set_y(d.b[1] * cMicroToTelsa);
			magnetic_field->set_z(d.b[2] * cMicroToTelsa);

			// Publish the messages
			imu_writer_->Write(imu);
		}
	}
	AINFO << "event_value.component.handle != 1";

	return true;
}

bool LPMSDriverComponent::Init(std::shared_ptr<apollo::cyber::Node> node)
{
	node_ = node;
	imu_writer_ = node_->CreateWriter<apollo::drivers::Imu>("/diamond/sensor/imu");

	auto clientPair = zen::make_client();
	m_zenClient = std::unique_ptr<zen::ZenClient>(new zen::ZenClient(std::move(clientPair.second)));

	if (clientPair.first != ZenError_None)
	{
		AERROR << "Cannot start OpenZen";
		return false;
	}

	if (m_openzenVerbose)
		ZenSetLogLevel(ZenLogLevel_Debug);
	else
		ZenSetLogLevel(ZenLogLevel_Off);

	// no sensor name given, auto-discovery
	if (m_sensorName.size() == 0)
	{
		AINFO << "OpenZen sensors will be listed";
		ZenError listError = m_zenClient->listSensorsAsync();

		if (listError != ZenError_None)
		{
			AERROR << "Cannot list sensors";
			return false;
		}

		bool listingDone = false;
		bool firstSensorFound = false;
		ZenSensorDesc foundSens;

		while (listingDone == false)
		{
			const auto pair = m_zenClient->waitForNextEvent();
			const bool success = pair.first;
			auto& event = pair.second;
			if (!success) break;

			if (!event.component.handle)
			{
				switch (event.eventType)
				{
				case ZenSensorEvent_SensorFound:
					if (!firstSensorFound)
					{
						foundSens = event.data.sensorFound;
						firstSensorFound = true;
					}
					AINFO << "OpenZen sensor with name "
					      << event.data.sensorFound.serialNumber
					      << " on IO system found" << event.data.sensorFound.ioType;
					break;

				case ZenSensorEvent_SensorListingProgress:
					if (event.data.sensorListingProgress.progress == 1.0f)
						listingDone = true;
					break;
				}
			}
		}

		if (!firstSensorFound)
		{
			AERROR << "No OpenZen sensors found";
			return false;
		}

		AINFO << "Connecting to found sensor " << foundSens.serialNumber
		      << " on IO system " << foundSens.ioType;
		// if a baudRate has been set, override the default given by OpenZen
		// listing
		if (m_baudrate > 0)
			foundSens.baudRate = m_baudrate;

		auto sensorObtainPair = m_zenClient->obtainSensor(foundSens);

		if (sensorObtainPair.first != ZenSensorInitError_None)
		{
			AERROR << "Cannot connect to sensor found with discovery. Make sure "
			       "you have the user rights to access serial devices.";
			return false;
		}
		m_zenSensor = std::unique_ptr<zen::ZenSensor>(new zen::ZenSensor(std::move(sensorObtainPair.second)));
	}
	else
	{
		// directly connect to sensor
		AINFO << "Connecting directly to sensor " << m_sensorName
		      << " over interface " << m_sensorInterface;
		auto sensorObtainPair = m_zenClient->obtainSensorByName(m_sensorInterface, m_sensorName, m_baudrate);

		if (sensorObtainPair.first != ZenSensorInitError_None)
		{
			AERROR << "Cannot connect directly to sensor.  Make sure you have the "
			       "user rights to access serial devices.";
			return false;
		}
		m_zenSensor = std::unique_ptr<zen::ZenSensor>(
		                  new zen::ZenSensor(std::move(sensorObtainPair.second)));
	}
	
	std::packaged_task<bool(LPMSDriverComponent::SensorThreadParams)> SensorPackage(m_sensorThread);	
	std::thread t_read(std::ref(SensorPackage)/*, param*/);
	t_read.detach();
	if(t_read.joinable())
		t_read.join();
	assert(!t_read.joinable());
	std::future<bool> SensorFuture = SensorPackage.get_future();
	
}


bool LPMSDriverComponent::run(void)
{
	if (!m_zenClient)
	{
		AERROR << "OpenZen could not be started";
		return false;
	}

	if (!m_zenSensor)
	{
		AERROR << "OpenZen sensor could not be connected";
		return false;
	}

	auto imuPair = m_zenSensor->getAnyComponentOfType(g_zenSensorType_Imu);
	auto& hasImu = imuPair.first;
	if (!hasImu)
	{
		// error, this sensor does not have an IMU component
		AINFO << "No IMU component available, sensor control commands won't be available";
	}
	else
	{
		m_zenImu = std::unique_ptr<zen::ZenSensorComponent>(new zen::ZenSensorComponent(std::move(imuPair.second)));
		publishIsAutocalibrationActive();
	}

	m_sensorThread.start(LPMSDriverComponent::SensorThreadParams{m_zenClient.get(), frame_id, imu_writer_, m_useLpmsAccelerationConvention});
	AINFO << "Data streaming from sensor started";

	return true;
}

void LPMSDriverComponent::publishIsAutocalibrationActive()
{
	Request* msg;

	if (!m_zenImu)
	{
		AINFO << "No IMU compontent available, can't publish autocalibration status";
		return;
	}

	auto resPair = m_zenImu->getBoolProperty(ZenImuProperty_GyrUseAutoCalibration);
	auto error = resPair.first;
	auto useAutoCalibration = resPair.second;
	if (error)
		AINFO << "get autocalibration Error";
	else
		msg->set_data(useAutoCalibration);
	// autocalibration_status_pub.publish(msg);
}

bool LPMSDriverComponent::setAutocalibration(const std::shared_ptr<Request>& req, const std::shared_ptr<Content>& res)
{
	AINFO << "set_autocalibration";

	std::string msg;

	if (!m_zenImu)
	{
		AINFO << "No IMU compontent available, can't set autocalibration status";
		return false;
	}

	if (auto error = m_zenImu->setBoolProperty(
	                     ZenImuProperty_GyrUseAutoCalibration, req->data()))
	{
		AINFO << "set autocalibration Error : " << error;
		res->set_success(false);
		msg.append(
		    std::string("[Failed] current autocalibration status set to: ") +
		    (req->data() ? "True" : "False"));

	}
	else
	{
		res->set_success(true);
		msg.append(std::string("[Success] autocalibration status set to: ") +
		           (req->data() ? "True" : "False"));
	}

	publishIsAutocalibrationActive();
	res->set_message(msg);

	return res->success();
}

bool LPMSDriverComponent::resetHeading(const std::shared_ptr<Content>& request,
                                       std::shared_ptr<Content>& response)
{
	if (!m_zenImu)
	{
		AINFO << "No IMU compontent available, can't reset heading";
		return false;
	}

	AINFO << "reset_heading";
	// Offset reset parameters:
	// 0: Object reset
	// 1: Heading reset
	// 2: Alignment reset
	if (auto error = m_zenImu->setInt32Property(
	                     ZenImuProperty_OrientationOffsetMode, 1))
	{
		AINFO << "Error: " << error;
		response->set_success(false);
		response->set_message("[Failed] Heading reset");
	}
	else
	{
		response->set_success(true);
		response->set_message("[Success] Heading reset");
	}
	return response->success();
}

bool LPMSDriverComponent::calibrateGyroscope(const std::shared_ptr<Content>& req,
        const std::shared_ptr<Content>& res)
{
	if (!m_zenImu)
	{
		AINFO << "No IMU compontent available, can't start autocalibration";
		return false;
	}

	AINFO << "calibrate_gyroscope: Please make sure the sensor is stationary for 4 seconds";

	if (auto error = m_zenImu->executeProperty(ZenImuProperty_CalibrateGyro))
	{
		AINFO << "Error : " << error;
		res->set_success(false);
		res->set_message("[Failed] Gyroscope calibration procedure error");
	}
	else
	{
		// ros::Duration(4).sleep();
		// TODO
		// apollo::cyber::Duration(4)::Sleep();
		res->set_success(true);
		res->set_message("[Success] Gyroscope calibration procedure completed");
		AINFO << "calibrate_gyroscope: Gyroscope calibration procedure completed";
	}
	return res->success();
}


}  // namespace LPMS
}  // namespace drivers
}  // namespace apollo
