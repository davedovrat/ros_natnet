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

#ifndef _NatNet_Tf_Broadcaster_H_
#define _NatNet_Tf_Broadcaster_H_

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include <memory>
#include <iostream>
#include <map>

namespace mocap {

/*
 * A NatNet client ROS node that broadcasts to the tf buffer.
 */

class NatNetTfBroadcaster : public rclcpp::Node
{
public:

	/********* Types *********/

	typedef enum
	{
		None=0,
		ServerSearching,
		ServerFound,
		NewRigidBody,
		NormalOperation
	}State;

	/* Constructor:
	 * strNodeName: ROS node name
	 */
	NatNetTfBroadcaster(std::string strNodeName = "mocap_broadcaster");

	~NatNetTfBroadcaster();

	void SetConnectParams(const sNatNetClientConnectParams &natNetClientConnectParams);
	void Connect();
	ErrorCode GetServerDescription();
	void GetDataDescriptions();
	void LogServerDescription();
	void SendRootStaticTransform();
	void SendStaticTransform(geometry_msgs::msg::TransformStamped &tf_msg){m_static_tf_broadcaster->sendTransform(tf_msg);}
	void SendTransform(geometry_msgs::msg::TransformStamped &tf_msg){m_tf_broadcaster->sendTransform(tf_msg);}
	sRigidBodyDescription* GetRigidBodyDescription(int nId);
	void SetFrameCallbackHandler(){m_pNatNetClient->SetFrameReceivedCallback(DataHandler, this);}
	void PeriodicHandler();
	void SetState(NatNetTfBroadcaster::State eState);
	void SetRootFrame(std::string strRootFrame){m_strRootFrameName = strRootFrame;}
	std::string GetRootFrameName(){return m_strRootFrameName;}

private:

	/********* Methods *********/

	// NATNET API procedures
	static void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);
	static void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext);

	/********* Attributes *********/

	NatNetTfBroadcaster::State m_eState;
	NatNetClient* m_pNatNetClient;
	NatNetDiscoveryHandle m_hDiscovery;
	sNatNetClientConnectParams m_sConnectParams;
	sServerDescription* m_pServerDescription;
	sDataDescriptions* m_pDataDefs;
	std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster;
	rclcpp::TimerBase::SharedPtr m_timer;
	std::string m_strRootFrameName;
	std::map<int32_t, int> m_mapIdIndex;
};

} // namespace mocap

#endif // _NatNet_Tf_Broadcaster_H_
