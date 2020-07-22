//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZenRos driver, under the MIT License.
// See the LICENSE file in the top-most folder for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

// #include "ros/ros.h"
// #include "sensor_msgs/Imu.h"
// #include "sensor_msgs/MagneticField.h"
// #include "std_srvs/SetBool.h"
// #include "std_srvs/Trigger.h"
// #include "std_msgs/Bool.h"

#include <memory>
#include <string>

#include "ManagedThread.h"
#include <OpenZen.h>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"
#include "cyber/common/log.h"

#include "modules/drivers/proto/imu.pb.h"


class OpenZenSensor
{
public:
    // Access to ROS node
    ros::NodeHandle nh, private_nh;
    ros::Timer updateTimer;

    // Publisher
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher autocalibration_status_pub;

    // Service
    ros::ServiceServer autocalibration_serv;
    ros::ServiceServer gyrocalibration_serv;
    ros::ServiceServer resetHeading_serv;
    
    // Parameters
    std::string m_sensorName;
    std::string m_sensorInterface;
    std::string frame_id;
    int m_baudrate = 0;


    OpenZenSensor(ros::NodeHandle h): 
        nh(h),
        private_nh("~"), 
        m_sensorThread( [](SensorThreadParams const& param) -> bool {

            const float cDegToRad = 3.1415926f/180.0f;
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
                        ROS_INFO("OpenZen sensor disconnected");
                        return false;
                }
            }

            if (event_value.component.handle == 1)
            {
                if (event_value.eventType == ZenImuEvent_Sample)
                {
                    // IMU
                    auto const& d = event_value.data.imuData;

                    sensor_msgs::Imu imu_msg;
                    sensor_msgs::MagneticField mag_msg;

                    // We follow this ROS conventions
                    // https://www.ros.org/reps/rep-0103.html
                    // https://www.ros.org/reps/rep-0145.html

                    imu_msg.header.stamp = ros::Time::now();
                    imu_msg.header.frame_id = param.frame_id;

                    // Fill orientation quaternion
                    imu_msg.orientation.w = d.q[0];
                    imu_msg.orientation.x = -d.q[1];
                    imu_msg.orientation.y = -d.q[2];
                    imu_msg.orientation.z = -d.q[3];

                    // Fill angular velocity data
                    // - scale from deg/s to rad/s
                    imu_msg.angular_velocity.x = d.g[0] * cDegToRad;
                    imu_msg.angular_velocity.y = d.g[1] * cDegToRad;
                    imu_msg.angular_velocity.z = d.g[2] * cDegToRad;

                    // Fill linear acceleration data
                    const float rosConversion = -1.0 * (!param.useLpmsAccelerationConvention) +
                        1.0 * param.useLpmsAccelerationConvention;

                    imu_msg.linear_acceleration.x = rosConversion * d.a[0] * cEarthG;
                    imu_msg.linear_acceleration.y = rosConversion * d.a[1] * cEarthG;
                    imu_msg.linear_acceleration.z = rosConversion * d.a[2] * cEarthG;

                    mag_msg.header.stamp = imu_msg.header.stamp;
                    mag_msg.header.frame_id = param.frame_id;

                    // Units are microTesla in the LPMS library, Tesla in ROS.
                    mag_msg.magnetic_field.x = d.b[0] * cMicroToTelsa;
                    mag_msg.magnetic_field.y = d.b[1] * cMicroToTelsa;
                    mag_msg.magnetic_field.z = d.b[2] * cMicroToTelsa;

                    // Publish the messages
                    param.imu_pub.publish(imu_msg);
                    param.mag_pub.publish(mag_msg);
                }
            }
                
            return true;
        })
    {
        // Get node parameters
        private_nh.param<std::string>("sensor_name", m_sensorName, "");
        private_nh.param<std::string>("sensor_interface", m_sensorInterface, "LinuxDevice");
        private_nh.param<bool>("openzen_verbose", m_openzenVerbose, false);
        // using 0 as default will tell OpenZen to use the defaul baudrate for a respective sensor
        private_nh.param("baudrate", m_baudrate, 0);

        // In LP-Research sensor output, the linear acceleration measurement is pointing down (z-) when
        // the sensor is lying flat on the table. ROS convention is z+ pointing up in this case
        // By default, this ROS driver converts to the ROS convention. Set this flag to true to
        // use the LPMS convention
        private_nh.param<bool>("use_lpms_acceleration_convention", m_useLpmsAccelerationConvention, false);
        private_nh.param<std::string>("frame_id", frame_id, "imu");

        // Publisher
        imu_pub = nh.advertise<sensor_msgs::Imu>("data",1);
        mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag",1);
        autocalibration_status_pub = nh.advertise<std_msgs::Bool>("is_autocalibration_active", 1, true);

        // Services
        autocalibration_serv = nh.advertiseService("enable_gyro_autocalibration", &OpenZenSensor::setAutocalibration, this);
        gyrocalibration_serv = nh.advertiseService("calibrate_gyroscope", &OpenZenSensor::calibrateGyroscope, this);
        resetHeading_serv = nh.advertiseService("reset_heading", &OpenZenSensor::resetHeading, this);


        auto clientPair = zen::make_client();
        m_zenClient = std::unique_ptr<zen::ZenClient>(new zen::ZenClient(std::move(clientPair.second)));

        if (clientPair.first != ZenError_None)
        {
            ROS_ERROR("Cannot start OpenZen");
            return;
        }

        if (m_openzenVerbose)
        {
            ZenSetLogLevel(ZenLogLevel_Debug);
        } 
        else
        {
            ZenSetLogLevel(ZenLogLevel_Off);
        }

        // no sensor name given, auto-discovery
        if (m_sensorName.size() == 0) 
        {
            ROS_INFO_STREAM("OpenZen sensors will be listed");
            ZenError listError = m_zenClient->listSensorsAsync();

            if (listError != ZenError_None)
            {
                ROS_ERROR("Cannot list sensors");
                return;
            }
            
            bool listingDone = false;
            bool firstSensorFound = false;
            ZenSensorDesc foundSens;

            while (listingDone == false)
            {
                const auto pair = m_zenClient->waitForNextEvent();
                const bool success = pair.first;
                auto& event = pair.second;
                if (!success)
                    break;

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
                        ROS_INFO_STREAM("OpenZen sensor with name " << event.data.sensorFound.serialNumber << " on IO system found" <<
                            event.data.sensorFound.ioType);
                        break;

                    case ZenSensorEvent_SensorListingProgress:
                        if (event.data.sensorListingProgress.progress == 1.0f)
                        {
                            listingDone = true;
                        }
                            
                        break;
                    }
                }
            }

            if (!firstSensorFound)
            {
                ROS_ERROR("No OpenZen sensors found");
                return;
            }

            ROS_INFO_STREAM("Connecting to found sensor " << foundSens.serialNumber << " on IO system " << foundSens.ioType);
            // if a baudRate has been set, override the default given by OpenZen listing
            if (m_baudrate > 0) {
                foundSens.baudRate = m_baudrate;
            }

            auto sensorObtainPair = m_zenClient->obtainSensor(foundSens);

            if (sensorObtainPair.first != ZenSensorInitError_None)
            {
                ROS_ERROR("Cannot connect to sensor found with discovery. Make sure you have the user rights to access serial devices.");
                return;
            }
            m_zenSensor = std::unique_ptr<zen::ZenSensor>( new zen::ZenSensor(std::move(sensorObtainPair.second)));
        } 
        else
        {
            // directly connect to sensor
            ROS_INFO_STREAM("Connecting directly to sensor " << m_sensorName << " over interface " << m_sensorInterface);
            auto sensorObtainPair = m_zenClient->obtainSensorByName(m_sensorInterface, m_sensorName, m_baudrate);

            if (sensorObtainPair.first != ZenSensorInitError_None)
            {
                ROS_ERROR("Cannot connect directly to sensor.  Make sure you have the user rights to access serial devices.");
                return;
            }
            m_zenSensor = std::unique_ptr<zen::ZenSensor>( new zen::ZenSensor(std::move(sensorObtainPair.second)));
        }

    }

    bool run(void)
    {
        if (!m_zenClient)
        {
            ROS_ERROR("OpenZen could not be started");
            return false;
        }

        if (!m_zenSensor)
        {
            ROS_ERROR("OpenZen sensor could not be connected");
            return false;
        }


        auto imuPair = m_zenSensor->getAnyComponentOfType(g_zenSensorType_Imu);
        auto& hasImu = imuPair.first;
        if (!hasImu)
        {
            // error, this sensor does not have an IMU component
            ROS_INFO("No IMU component available, sensor control commands won't be available");
        } else {
            m_zenImu = std::unique_ptr<zen::ZenSensorComponent>( new zen::ZenSensorComponent(std::move(imuPair.second)));
            publishIsAutocalibrationActive();
        }

        m_sensorThread.start( SensorThreadParams{
            m_zenClient.get(),
            frame_id,
            imu_pub,
            mag_pub,
            m_useLpmsAccelerationConvention
        } );

        ROS_INFO("Data streaming from sensor started");

        return true;
    }


    ///////////////////////////////////////////////////
    // Service Callbacks
    ///////////////////////////////////////////////////
    void publishIsAutocalibrationActive()
    {
        std_msgs::Bool msg;

        if (!m_zenImu) {
            ROS_INFO("No IMU compontent available, can't publish autocalibration status");
            return;
        }

        auto resPair = m_zenImu->getBoolProperty(ZenImuProperty_GyrUseAutoCalibration);
        auto error = resPair.first;
        auto useAutoCalibration = resPair.second;
        if (error) 
        {
            ROS_INFO("get autocalibration Error");
        }
        else 
        {
            msg.data = useAutoCalibration;
            autocalibration_status_pub.publish(msg);   
        }
    }

    bool setAutocalibration (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        ROS_INFO("set_autocalibration");

        std::string msg;

        if (!m_zenImu) {
            ROS_INFO("No IMU compontent available, can't set autocalibration status");
            return false;
        }

        if (auto error = m_zenImu->setBoolProperty(ZenImuProperty_GyrUseAutoCalibration, req.data))
        {
            ROS_INFO("set autocalibration Error");
            res.success = false; 
            msg.append(std::string("[Failed] current autocalibration status set to: ") + (req.data?"True":"False"));
        
        }
        else
        {
            res.success = true;
            msg.append(std::string("[Success] autocalibration status set to: ") + (req.data?"True":"False"));
        }

        publishIsAutocalibrationActive();        
        res.message = msg;

        return res.success;
    }

    bool resetHeading (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if (!m_zenImu) {
            ROS_INFO("No IMU compontent available, can't reset heading");
            return false;
        }

        ROS_INFO("reset_heading");
        // Offset reset parameters:
        // 0: Object reset
        // 1: Heading reset
        // 2: Alignment reset
        if (auto error = m_zenImu->setInt32Property( ZenImuProperty_OrientationOffsetMode, 1)) 
        {
            ROS_INFO("Error");
            res.success = false;
            res.message = "[Failed] Heading reset";
        } 
        else 
        {
            res.success = true;
            res.message = "[Success] Heading reset";
        }
        return res.success;
    }


    bool calibrateGyroscope (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if (!m_zenImu) {
            ROS_INFO("No IMU compontent available, can't start autocalibration");
            return false;
        }

        ROS_INFO("calibrate_gyroscope: Please make sure the sensor is stationary for 4 seconds");

        if (auto error = m_zenImu->executeProperty(ZenImuProperty_CalibrateGyro))
        {
            ROS_INFO("Error");

            res.success = false;
            res.message = "[Failed] Gyroscope calibration procedure error";
        }
        else
        {
            ros::Duration(4).sleep();
            res.success = true;
            res.message = "[Success] Gyroscope calibration procedure completed";
            ROS_INFO("calibrate_gyroscope: Gyroscope calibration procedure completed");

        }
        return res.success;
    }


 private:

    std::unique_ptr<zen::ZenClient> m_zenClient;
    std::unique_ptr<zen::ZenSensor> m_zenSensor;
    std::unique_ptr<zen::ZenSensorComponent> m_zenImu;

    bool m_openzenVerbose;
    bool m_useLpmsAccelerationConvention;

    struct SensorThreadParams
    {
        zen::ZenClient * zenClient;
        std::string frame_id;
        ros::Publisher & imu_pub;
        ros::Publisher & mag_pub;
        bool useLpmsAccelerationConvention;
    };

    ManagedThread<SensorThreadParams> m_sensorThread;
};

int main(int argc, char *argv[])
{
    apollo::cyber::init(argc, argv, "openzen_sensor_node");
    std::shared_ptr<apollo::cyber::Node> node(
      apollo::cyber::CreateNode("openzen"));
    
    // ros::AsyncSpinner spinner(0);
    // spinner.start();

    OpenZenSensor lpOpenZen(node);

    if (!lpOpenZen.run())
    {
        apollo::cyber::AsyncShutdown();
        return 1;
    }

    // ros::waitForShutdown();
    apollo::cyber::WaitForShutdown();

    AINFO << "OpenZenNode exit done.";

    return 0;
}
