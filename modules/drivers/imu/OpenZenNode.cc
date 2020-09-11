//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZenRos driver, under the MIT License.
// See the LICENSE file in the top-most folder for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#include <memory>
#include <string>

#include <OpenZen.h>
#include "ManagedThread.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/drivers/imu/proto/service.pb.h"
#include "modules/drivers/proto/imu.pb.h"

using apollo::drivers::LPMS::Content;
using apollo::drivers::LPMS::Request;

class OpenZenSensor {
 public:
  // Access to node
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

  OpenZenSensor(std::shared_ptr<apollo::cyber::Node> node)
      : node_(node),
        m_sensorThread([&](SensorThreadParams const& param) -> bool {
          const float cDegToRad = 3.1415926f / 180.0f;
          const float cEarthG = 9.81f;
          const float cMicroToTelsa = 1e-6f;

          auto event = param.zenClient->waitForNextEvent();
          auto have_event = event.first;
          auto event_value = event.second;

          if (!have_event) {
            // empty event received, terminate
            return false;
          }

          if (!event_value.component.handle) {
            // not an event from a component
            switch (event_value.eventType) {
              case ZenSensorEvent_SensorDisconnected:
                AINFO << "OpenZen sensor disconnected";
                return false;
            }
          }

          if (event_value.component.handle == 1) {
            AINFO << "event_value.component.handle == 1";
            if (event_value.eventType == ZenImuEvent_Sample) {
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

              //   // Fill angular velocity data
              //   // - scale from deg/s to rad/s
              auto angular_velocity = imu.mutable_angular_velocity();
              angular_velocity->set_x(d.g[0] * cDegToRad);
              angular_velocity->set_y(d.g[1] * cDegToRad);
              angular_velocity->set_z(d.g[2] * cDegToRad);

              //   // Fill linear acceleration data
              const float rosConversion =
                  -1.0 * (!param.useLpmsAccelerationConvention) +
                  1.0 * param.useLpmsAccelerationConvention;

              auto linear_acceleration = imu.mutable_linear_acceleration();
              linear_acceleration->set_x(rosConversion * d.a[0] * cEarthG);
              linear_acceleration->set_y(rosConversion * d.a[1] * cEarthG);
              linear_acceleration->set_z(rosConversion * d.a[2] * cEarthG);

              //   // Units are microTesla in the LPMS library, Tesla in ROS.
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
        }) {
    // Get node parameters
    // private_nh.param<std::string>("sensor_name", m_sensorName, "");
    // private_nh.param<std::string>("sensor_interface", m_sensorInterface,
    // "LinuxDevice"); private_nh.param<bool>("openzen_verbose",
    // m_openzenVerbose, false); using 0 as default will tell OpenZen to use the
    // defaul baudrate for a respective sensor private_nh.param("baudrate",
    // m_baudrate, 0);

    // In LP-Research sensor output, the linear acceleration measurement is
    // pointing down (z-) when the sensor is lying flat on the table. ROS
    // convention is z+ pointing up in this case By default, this ROS driver
    // converts to the ROS convention. Set this flag to true to use the LPMS
    // convention private_nh.param<bool>("use_lpms_acceleration_convention",
    // m_useLpmsAccelerationConvention, false);
    // private_nh.param<std::string>("frame_id", frame_id, "imu");

    // Publisher
    // imu_pub = nh.advertise<sensor_msgs::Imu>("data",1);
    // mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag",1);
    // autocalibration_status_pub =
    // nh.advertise<std_msgs::Bool>("is_autocalibration_active", 1, true);
    imu_writer_ =
        node_->CreateWriter<apollo::drivers::Imu>(FLAGS_raw_imu_topic);

    // Services
    // autocalibration_serv = nh.advertiseService("enable_gyro_autocalibration",
    // &OpenZenSensor::setAutocalibration, this); gyrocalibration_serv =
    // nh.advertiseService("calibrate_gyroscope",
    // &OpenZenSensor::calibrateGyroscope, this); resetHeading_serv =
    // nh.advertiseService("reset_heading", &OpenZenSensor::resetHeading, this);

    // auto autocalibration_serv = node_->CreateService<Content,
    // Content>("enable_gyro_autocalibration",
    // &OpenZenSensor::setAutocalibration); auto gyrocalibration_serv =
    // node_->CreateService<Content, Content>("calibrate_gyroscope",
    // &OpenZenSensor::calibrateGyroscope); auto resetHeading_serv =
    // node_->CreateService<Content, Content>("reset_heading",
    // &OpenZenSensor::resetHeading);

    auto clientPair = zen::make_client();
    m_zenClient = std::unique_ptr<zen::ZenClient>(
        new zen::ZenClient(std::move(clientPair.second)));

    if (clientPair.first != ZenError_None) {
      AERROR << "Cannot start OpenZen";
      return;
    }

    if (m_openzenVerbose) {
      ZenSetLogLevel(ZenLogLevel_Debug);
    } else {
      ZenSetLogLevel(ZenLogLevel_Off);
    }

    // no sensor name given, auto-discovery
    if (m_sensorName.size() == 0) {
      AINFO << "OpenZen sensors will be listed";
      ZenError listError = m_zenClient->listSensorsAsync();

      if (listError != ZenError_None) {
        AERROR << "Cannot list sensors";
        return;
      }

      bool listingDone = false;
      bool firstSensorFound = false;
      ZenSensorDesc foundSens;

      while (listingDone == false) {
        const auto pair = m_zenClient->waitForNextEvent();
        const bool success = pair.first;
        auto& event = pair.second;
        if (!success) break;

        if (!event.component.handle) {
          switch (event.eventType) {
            case ZenSensorEvent_SensorFound:
              if (!firstSensorFound) {
                foundSens = event.data.sensorFound;
                firstSensorFound = true;
              }
              AINFO << "OpenZen sensor with name "
                    << event.data.sensorFound.serialNumber
                    << " on IO system found" << event.data.sensorFound.ioType;
              break;

            case ZenSensorEvent_SensorListingProgress:
              if (event.data.sensorListingProgress.progress == 1.0f) {
                listingDone = true;
              }

              break;
          }
        }
      }

      if (!firstSensorFound) {
        AERROR << "No OpenZen sensors found";
        return;
      }

      AINFO << "Connecting to found sensor " << foundSens.serialNumber
            << " on IO system " << foundSens.ioType;
      // if a baudRate has been set, override the default given by OpenZen
      // listing
      if (m_baudrate > 0) {
        foundSens.baudRate = m_baudrate;
      }

      auto sensorObtainPair = m_zenClient->obtainSensor(foundSens);

      if (sensorObtainPair.first != ZenSensorInitError_None) {
        AERROR << "Cannot connect to sensor found with discovery. Make sure "
                  "you have the user rights to access serial devices.";
        return;
      }
      m_zenSensor = std::unique_ptr<zen::ZenSensor>(
          new zen::ZenSensor(std::move(sensorObtainPair.second)));
    } else {
      // directly connect to sensor
      AINFO << "Connecting directly to sensor " << m_sensorName
            << " over interface " << m_sensorInterface;
      auto sensorObtainPair = m_zenClient->obtainSensorByName(
          m_sensorInterface, m_sensorName, m_baudrate);

      if (sensorObtainPair.first != ZenSensorInitError_None) {
        AERROR << "Cannot connect directly to sensor.  Make sure you have the "
                  "user rights to access serial devices.";
        return;
      }
      m_zenSensor = std::unique_ptr<zen::ZenSensor>(
          new zen::ZenSensor(std::move(sensorObtainPair.second)));
    }
  }

  bool run(void) {
    if (!m_zenClient) {
      AERROR << "OpenZen could not be started";
      return false;
    }

    if (!m_zenSensor) {
      AERROR << "OpenZen sensor could not be connected";
      return false;
    }

    auto imuPair = m_zenSensor->getAnyComponentOfType(g_zenSensorType_Imu);
    auto& hasImu = imuPair.first;
    if (!hasImu) {
      // error, this sensor does not have an IMU component
      AINFO << "No IMU component available, sensor control commands won't be "
               "available";
    } else {
      m_zenImu = std::unique_ptr<zen::ZenSensorComponent>(
          new zen::ZenSensorComponent(std::move(imuPair.second)));
      publishIsAutocalibrationActive();
    }

    m_sensorThread.start(SensorThreadParams{m_zenClient.get(), frame_id,
                                            imu_writer_,
                                            m_useLpmsAccelerationConvention});

    AINFO << "Data streaming from sensor started";

    return true;
  }

  ///////////////////////////////////////////////////
  // Service Callbacks
  ///////////////////////////////////////////////////
  void publishIsAutocalibrationActive() {
    Request* msg;

    if (!m_zenImu) {
      AINFO << "No IMU compontent available, can't publish autocalibration "
               "status";
      return;
    }

    auto resPair =
        m_zenImu->getBoolProperty(ZenImuProperty_GyrUseAutoCalibration);
    auto error = resPair.first;
    auto useAutoCalibration = resPair.second;
    if (error) {
      AINFO << "get autocalibration Error";
    } else {
      msg->set_data(useAutoCalibration);
      // autocalibration_status_pub.publish(msg);
    }
  }

  bool setAutocalibration(const std::shared_ptr<Request>& req,
                          const std::shared_ptr<Content>& res) {
    AINFO << "set_autocalibration";

    std::string msg;

    if (!m_zenImu) {
      AINFO << "No IMU compontent available, can't set autocalibration status";
      return false;
    }

    if (auto error = m_zenImu->setBoolProperty(
            ZenImuProperty_GyrUseAutoCalibration, req->data())) {
      AINFO << "set autocalibration Error : " << error;
      res->set_success(false);
      msg.append(
          std::string("[Failed] current autocalibration status set to: ") +
          (req->data() ? "True" : "False"));

    } else {
      res->set_success(true);
      msg.append(std::string("[Success] autocalibration status set to: ") +
                 (req->data() ? "True" : "False"));
    }

    publishIsAutocalibrationActive();
    res->set_message(msg);

    return res->success();
  }

  bool resetHeading(const std::shared_ptr<Content>& request,
                    std::shared_ptr<Content>& response) {
    if (!m_zenImu) {
      AINFO << "No IMU compontent available, can't reset heading";
      return false;
    }

    AINFO << "reset_heading";
    // Offset reset parameters:
    // 0: Object reset
    // 1: Heading reset
    // 2: Alignment reset
    if (auto error = m_zenImu->setInt32Property(
            ZenImuProperty_OrientationOffsetMode, 1)) {
      AINFO << "Error: " << error;
      response->set_success(false);
      response->set_message("[Failed] Heading reset");
    } else {
      response->set_success(true);
      response->set_message("[Success] Heading reset");
    }
    return response->success();
  }

  bool calibrateGyroscope(const std::shared_ptr<Content>& req,
                          const std::shared_ptr<Content>& res) {
    if (!m_zenImu) {
      AINFO << "No IMU compontent available, can't start autocalibration";
      return false;
    }

    AINFO << "calibrate_gyroscope: Please make sure the sensor is stationary "
             "for 4 seconds";

    if (auto error = m_zenImu->executeProperty(ZenImuProperty_CalibrateGyro)) {
      AINFO << "Error : " << error;

      res->set_success(false);
      res->set_message("[Failed] Gyroscope calibration procedure error");
    } else {
      // ros::Duration(4).sleep();
      // TODO
      // apollo::cyber::Duration(4)::Sleep();
      res->set_success(true);
      res->set_message("[Success] Gyroscope calibration procedure completed");
      AINFO << "calibrate_gyroscope: Gyroscope calibration procedure completed";
    }
    return res->success();
  }

 private:
  std::unique_ptr<zen::ZenClient> m_zenClient;
  std::unique_ptr<zen::ZenSensor> m_zenSensor;
  std::unique_ptr<zen::ZenSensorComponent> m_zenImu;

  bool m_openzenVerbose;
  bool m_useLpmsAccelerationConvention;

  struct SensorThreadParams {
    zen::ZenClient* zenClient;
    std::string frame_id;
    std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Imu>> imu_writer_;
    bool useLpmsAccelerationConvention;
  };

  ManagedThread<SensorThreadParams> m_sensorThread;
};

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("openzen_node"));

  OpenZenSensor lpOpenZen(node);

  if (!lpOpenZen.run()) {
    // auto client_gyro = node->CreateClient<Content,
    // Content>("enable_gyro_autocalibration"); auto client_calibration =
    // node->CreateClient<Content, Content>("calibrate_gyroscope"); auto
    // client_reset_heading = node->CreateClient<Content,
    // Content>("reset_heading"); auto request = std::make_shared<Content>();
    // request->set_success(false);
    // request->set_message('init');
    // auto res = client_reset_heading->SendRequest(request);

    // if (res != nullptr) {
    //     AINFO << "client: responese: " << res->ShortDebugString();
    // } else {
    //     AINFO << "client: service may not ready.";
    // }
    apollo::cyber::AsyncShutdown();
    return 1;
  }

  apollo::cyber::WaitForShutdown();

  AINFO << "OpenZenNode exit done.";

  return 0;
}
