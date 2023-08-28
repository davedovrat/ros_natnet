/*
Copyright 2022 David Dovrat

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>

#include "ros_natnet/NatNetTfBroadcaster.h"

static std::shared_ptr<mocap::NatNetTfBroadcaster> pNatNetTfBroadcaster;

void NATNET_CALLCONV MessageHandler( Verbosity msgType, const char* msg )
{
	switch ( msgType )
	{
        case Verbosity_Debug:
        	RCLCPP_DEBUG(pNatNetTfBroadcaster->get_logger(), "NatNetTfBroadcaster:: '%s'", msg);
            break;
        case Verbosity_Info:
        	RCLCPP_INFO(pNatNetTfBroadcaster->get_logger(), "NatNetTfBroadcaster:: '%s'", msg);
            break;
        case Verbosity_Warning:
        	RCLCPP_WARN(pNatNetTfBroadcaster->get_logger(), "NatNetTfBroadcaster:: '%s'", msg);
            break;
        case Verbosity_Error:
        	RCLCPP_ERROR(pNatNetTfBroadcaster->get_logger(), "NatNetTfBroadcaster:: '%s'", msg);
            break;
        default:
        	RCLCPP_INFO(pNatNetTfBroadcaster->get_logger(), "NatNetTfBroadcaster['%d']:: '%s'", msgType, msg);
            break;
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // print NatNet API version info
  unsigned char ver[4];
  NatNet_GetVersion( ver );

  std::cout << "NatNet API version: " << (int)ver[0] << "."
		  << (int)ver[1] << "." << (int)ver[2] << "." << (int)ver[3] << "." << std::endl;

  std::cout << "thread: main " << std::this_thread::get_id() << std::endl;

  pNatNetTfBroadcaster = std::make_shared<mocap::NatNetTfBroadcaster>();

  // Install logging callback
  NatNet_SetLogCallback(MessageHandler);

  rclcpp::spin(pNatNetTfBroadcaster);

  rclcpp::shutdown();
  return 0;
}
