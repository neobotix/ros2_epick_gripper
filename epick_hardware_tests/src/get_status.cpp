// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "epick_driver/default_driver.hpp"
#include "epick_driver/default_serial.hpp"

#include "command_line_utility.hpp"

#include <map>
#include <memory>
#include <vector>
#include <iostream>

// With this test we connect to the gripper, activate it and read its status.

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 115200;
constexpr auto kTimeout = 500;  // milliseconds
constexpr auto kSlaveAddress = 0x09;

int main(int argc, char* argv[])
{
  CommandLineUtility cli;

  std::string port = kComPort;
  cli.registerHandler(
      "--port", [&port](const char* value) { port = value; }, false);

  int baudrate = kBaudRate;
  cli.registerHandler(
      "--baudrate", [&baudrate](const char* value) { baudrate = std::stoi(value); }, false);

  int timeout = kTimeout;
  cli.registerHandler(
      "--timeout", [&timeout](const char* value) { timeout = std::stoi(value); }, false);

  int slave_address = kSlaveAddress;
  cli.registerHandler(
      "--slave_address", [&slave_address](const char* value) { slave_address = std::stoi(value); }, false);

  cli.registerHandler("-h", []() {
    std::cout << "Usage: ./connect [OPTIONS]\n"
              << "Options:\n"
              << "  --port VALUE            Set the com port (default " << kComPort << ")\n"
              << "  --baudrate VALUE        Set the baudrate (default " << kBaudRate << "bps)\n"
              << "  --timeout VALUE         Set the read/write timeout (default " << kTimeout << "ms)\n"
              << "  --slave_address VALUE   Set the slave address (default " << kSlaveAddress << ")\n"
              << "  -h                      Show this help message\n";
    exit(0);
  });

  if (!cli.parse(argc, argv))
  {
    return 1;
  }

  try
  {
    auto serial = std::make_unique<epick_driver::DefaultSerial>();
    serial->set_port(port);
    serial->set_baudrate(baudrate);
    serial->set_timeout(timeout);

    auto driver = std::make_unique<epick_driver::DefaultDriver>(std::move(serial), slave_address);

    std::cout << "Using the following parameters: " << std::endl;
    std::cout << " - port: " << port << std::endl;
    std::cout << " - baudrate: " << baudrate << "bps" << std::endl;
    std::cout << " - read/write timeut: " << timeout << "ms" << std::endl;
    std::cout << " - slave address: " << slave_address << std::endl;

    std::cout << "Checking if the gripper is connected..." << std::endl;

    bool connected = driver->connect();
    if (!connected)
    {
      std::cout << "The gripper is not connected" << std::endl;
      return 1;
    }

    std::cout << "The gripper is connected." << std::endl;
    std::cout << "Activating the gripper..." << std::endl;

    driver->activate();

    std::cout << "The gripper is activated." << std::endl;
    std::cout << "Reading the gripper status..." << std::endl;

    epick_driver::GripperStatus status = driver->get_status();

    std::cout << "Status retrieved:" << std::endl;

    static std::map<epick_driver::GripperActivation, std::string> gripper_activation_names = {
      { epick_driver::GripperActivation::Inactive, "Inactive" },
      { epick_driver::GripperActivation::Active, "Active" },
    };

    static std::map<epick_driver::ObjectDetection, std::string> object_detection_names = {
      { epick_driver::ObjectDetection::Unknown, "Unknown" },
      { epick_driver::ObjectDetection::ObjectDetected, "ObjectDetected" },
      { epick_driver::ObjectDetection::NoObjectDetected, "NoObjectDetected" },
    };

    static std::map<epick_driver::GripperMode, std::string> gripper_mode_names = {
      { epick_driver::GripperMode::AutomaticMode, "AutomaticMode" },
      { epick_driver::GripperMode::AdvancedMode, "AdvancedMode" },
      { epick_driver::GripperMode::Reserved, "Reserved" }
    };

    std::cout << " - activation: " << gripper_activation_names.at(status.activation) << std::endl;
    std::cout << " - mode: " << gripper_mode_names.at(status.mode) << std::endl;
    std::cout << " - object detection: " << object_detection_names.at(status.object_detection) << std::endl;
    std::cout << " - actual pressure: " << status.pressure << "kPa" << std::endl;
  }
  catch (const serial::IOException& e)
  {
    std::cout << "Failed to communicating with the gripper:" << e.what();
    return 1;
  }

  return 0;
}
