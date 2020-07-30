//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#include "OpenZen.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <gsl/span>

std::vector<ZenSensorDesc> g_discoveredSensors;
std::condition_variable g_discoverCv;
std::mutex g_discoverMutex;
std::atomic_bool g_terminate(false);

std::atomic<uintptr_t> g_imuHandle;
std::atomic<uintptr_t> g_gnssHandle;

using namespace zen;

namespace
{
    void addDiscoveredSensor(const ZenEventData_SensorFound& desc)
    {
        std::lock_guard<std::mutex> lock(g_discoverMutex);
        g_discoveredSensors.push_back(desc);
    }
}

void pollLoop(std::reference_wrapper<ZenClient> client)
{
    while (!g_terminate)
    {
        unsigned int i = 0;
        while (true)
        {
            const auto pair = client.get().waitForNextEvent();
            const bool success = pair.first;
            auto& event = pair.second;
            if (!success)
                break;

            if (!event.component.handle)
            {
                switch (event.eventType)
                {
                case ZenSensorEvent_SensorFound:
                    addDiscoveredSensor(event.data.sensorFound);
                    break;

                case ZenSensorEvent_SensorListingProgress:
                    if (event.data.sensorListingProgress.progress == 1.0f)
                        g_discoverCv.notify_one();
                    break;
                }
            }
            else if (( g_imuHandle > 0) && (event.component.handle == g_imuHandle))
            {
                switch (event.eventType)
                {
                case ZenImuEvent_Sample:
                    if (i++ % 100 == 0) {
                        std::cout << "Event type: " << event.eventType << std::endl;
                        std::cout << "> Event component: " << uint32_t(event.component.handle) << std::endl;
                        std::cout << "> Acceleration: \t x = " << event.data.imuData.a[0]
                            << "\t y = " << event.data.imuData.a[1]
                            << "\t z = " << event.data.imuData.a[2] << std::endl;
                        std::cout << "> Gyro: \t\t x = " << event.data.imuData.g[0]
                            << "\t y = " << event.data.imuData.g[1]
                            << "\t z = " << event.data.imuData.g[2] << std::endl;
                    }
                    break;
                }
            }
            else if (( g_gnssHandle > 0) && (event.component.handle == g_gnssHandle))
            {
                switch (event.eventType)
                {
                case ZenGnssEvent_Sample:
                        std::cout << "Event type: " << event.eventType << std::endl;
                        std::cout << "> Event component: " << uint32_t(event.component.handle) << std::endl;
                        std::cout << "> GPS Fix: \t = " << event.data.gnssData.fixType << std::endl;
                        std::cout << "> Longitude: \t = " << event.data.gnssData.longitude
                            << "   Latitude: \t = " << event.data.gnssData.latitude << std::endl;
                        std::cout << " > GPS Time " << int(event.data.gnssData.year) << "/"
                            << int(event.data.gnssData.month) << "/"
                            << int(event.data.gnssData.day) << " "
                            << int(event.data.gnssData.hour) << ":"
                            << int(event.data.gnssData.minute) << ":"
                            << int(event.data.gnssData.second) << " UTC" << std::endl;

                        if (event.data.gnssData.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_None) {
                            std::cout << " > RTK not used" << std::endl;
                        }
                        else if (event.data.gnssData.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_FloatAmbiguities) {
                            std::cout << " > RTK used with float ambiguities" << std::endl;
                        }
                        else if (event.data.gnssData.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_FixedAmbiguities) {
                            std::cout << " > RTK used with fixed ambiguities" << std::endl;
                        }

                        break;
                }
            }
        }
    }

    std::cout << "--- Exit polling thread ---" << std::endl;
}

int main(int argc, char *argv[])
{
    ZenSetLogLevel(ZenLogLevel_Info);

    g_imuHandle = 0;
    g_gnssHandle = 0;

    auto clientPair = make_client();
    auto& clientError = clientPair.first;
    auto& client = clientPair.second;

    if (clientError)
        return clientError;

    std::thread pollingThread(&pollLoop, std::ref(client));

    std::cout << "Listing sensors:" << std::endl;

    if (auto error = client.listSensorsAsync())
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return error;
    }

    std::unique_lock<std::mutex> lock(g_discoverMutex);
    g_discoverCv.wait(lock);

    if (g_discoveredSensors.empty())
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return ZenError_Unknown;
    }

    for (unsigned idx = 0; idx < g_discoveredSensors.size(); ++idx)
        std::cout << idx << ": " << g_discoveredSensors[idx].name << " ("  << g_discoveredSensors[idx].ioType << ")" << std::endl;

    unsigned int idx;
    do
    {
        std::cout << "Provide an index within the range 0-" << g_discoveredSensors.size() - 1 << ":" << std::endl;
        std::cin >> idx;
    } while (idx >= g_discoveredSensors.size());
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    auto sensorPair = client.obtainSensor(g_discoveredSensors[idx]);
    auto& obtainError = sensorPair.first;
    auto& sensor = sensorPair.second;
    if (obtainError)
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return obtainError;
    }

    auto imuPair = sensor.getAnyComponentOfType(g_zenSensorType_Imu);
    auto& hasImu = imuPair.first;
    auto imu = imuPair.second;

    if (!hasImu)
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return ZenError_WrongSensorType;
    }

    // store the handle to the IMU to identify data coming from the imu
    // in our data processing thread
    g_imuHandle = imu.component().handle;

    // Get a string property
    auto sensorModelPair = sensor.getStringProperty(ZenSensorProperty_SensorModel);
    auto & sensorModelError = sensorModelPair.first;
    auto & sensorModelName = sensorModelPair.second;
    if (sensorModelError) {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return sensorModelError;
    }
    std::cout << "Sensor Model: " << sensorModelName << std::endl;

    // enable this to stream the sensor data to a network address
    // the ZEN_NETWORK build option needs to be enabled for this feature
    // to work.
    //sensor.publishEvents("tcp://*:8877");

    // check if a Gnss component is present on this sensor
    auto gnssPair = sensor.getAnyComponentOfType(g_zenSensorType_Gnss);
    auto& hasGnss = gnssPair.first;
    auto gnss = gnssPair.second;

    if (hasGnss)
    {
        // store the handle to the Gnss to identify data coming from the Gnss
        // in our data processing thread
        g_gnssHandle = gnss.component().handle;
        std::cout << "Gnss Component present on sensor" << std::endl;

        // enable this code for RTK forwarding from network
        /*
        if (gnss.forwardRtkCorrections("RTCM3Network", "192.168.1.117", 9000) != ZenError_None) {
            std::cout << "Cannot set RTK correction forwarding" << std::endl;
        }
        else {
            std::cout << "RTK correction forwarding started" << std::endl;
        }
        */

        // enable this code for RTK forwarding from serial port
        /*
        if (gnss.forwardRtkCorrections("RTCM3Serial", "COM11", 57600) != ZenError_None) {
            std::cout << "Cannot set RTK correction forwarding" << std::endl;
        }
        else {
            std::cout << "RTK correction forwarding started" << std::endl;
        }
        */
    }

    // Enable sensor streaming
    if (auto error = imu.setBoolProperty(ZenImuProperty_StreamData, true))
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return error;
    }

    std::string line;
    while (!g_terminate)
    {
        std::cout << "Type: " << std::endl;
        std::cout << " - 'q' to quit;" << std::endl;
        std::cout << " - 'r' to manually release the sensor;" << std::endl;

        if (std::getline(std::cin, line))
        {
            if (line == "q")
                g_terminate = true;
            else if (line == "r")
                sensor.release();
        }
    }

    client.close();
    pollingThread.join();
    return 0;
}
