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

#include "ros_natnet/NatNetTfBroadcaster.h"

#include<string>
#include<vector>
#include<chrono>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mocap {

//-----------------------------------------------------------------------------------------------------

NatNetTfBroadcaster::NatNetTfBroadcaster(std::string strNodeName) : rclcpp::Node(strNodeName)
{
	m_eState = NatNetTfBroadcaster::None;
	m_pNatNetClient = new NatNetClient();
    m_pServerDescription = new sServerDescription;
    m_pDataDefs = NULL;

    this->declare_parameter("root_frame", "mocap");
    this->declare_parameter("parent_frame", "odom");
    this->declare_parameter("static_tf_hz", 1);
    this->declare_parameter("location", std::vector<double>({0, 0, 0}));
    this->declare_parameter("orientation", std::vector<double>({0.5, 0.5, 0.5, 0.5}));

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    rclcpp::Parameter param_static_tf_hz = this->get_parameter("static_tf_hz");
    std::chrono::milliseconds ms_idle(1000/param_static_tf_hz.as_int());

    m_timer = this->create_wall_timer(ms_idle, std::bind(&NatNetTfBroadcaster::PeriodicHandler, this));

    // NatNet stuff

    if (NULL == m_pNatNetClient)
    {
    	throw std::runtime_error("NatNetTfBroadcaster:: could not create instantiate NatNet Client");
    }

	m_sConnectParams.connectionType = ConnectionType_Unicast;
	m_sConnectParams.multicastAddress = NULL;

	m_hDiscovery = NULL;
}

//-----------------------------------------------------------------------------------------------------

NatNetTfBroadcaster::~NatNetTfBroadcaster()
{
	m_pNatNetClient->Disconnect();
	NatNet_FreeAsyncServerDiscovery(m_hDiscovery);
	delete m_pNatNetClient;
	delete m_pServerDescription;

    if (m_pDataDefs)
    {
        NatNet_FreeDescriptions(m_pDataDefs);
    }
}

//-----------------------------------------------------------------------------------------------------

void NatNetTfBroadcaster::PeriodicHandler()
{
	switch(m_eState)
	{
	case NatNetTfBroadcaster::None:
		RCLCPP_DEBUG(get_logger(), "NatNetTfBroadcaster::PeriodicHandler: State None");
	    NatNet_CreateAsyncServerDiscovery( &m_hDiscovery, ServerDiscoveredCallback, this);
	    SetState(NatNetTfBroadcaster::ServerSearching);
		break;
	case NatNetTfBroadcaster::ServerSearching:
		RCLCPP_DEBUG(get_logger(), "NatNetTfBroadcaster::PeriodicHandler: State ServerSearching");
		break;
	case NatNetTfBroadcaster::ServerFound:
		RCLCPP_DEBUG(get_logger(), "NatNetTfBroadcaster::PeriodicHandler: State ServerFound");
		Connect();
		SetState(NatNetTfBroadcaster::NewRigidBody);
		break;
	case NatNetTfBroadcaster::NewRigidBody:
		RCLCPP_INFO(get_logger(), "NatNetTfBroadcaster::PeriodicHandler: State NewRigidBody");
		GetDataDescriptions();
		SetState(NatNetTfBroadcaster::NormalOperation);
		break;
	case NatNetTfBroadcaster::NormalOperation:
		RCLCPP_DEBUG(get_logger(), "NatNetTfBroadcaster::PeriodicHandler: State NormalOperation");
		SendRootStaticTransform();
		break;
	default:
		break;
	}
}

//-----------------------------------------------------------------------------------------------------

void NatNetTfBroadcaster::SetState(NatNetTfBroadcaster::State eState)
{
	m_eState = eState;
	switch(m_eState)
	{
	case NatNetTfBroadcaster::None:
		RCLCPP_INFO(get_logger(), "NatNetTfBroadcaster::SetState: None");
		break;
	case NatNetTfBroadcaster::ServerSearching:
		RCLCPP_INFO(get_logger(), "NatNetTfBroadcaster::SetState: ServerSearching");
		break;
	case NatNetTfBroadcaster::ServerFound:
		RCLCPP_INFO(get_logger(), "NatNetTfBroadcaster::SetState: ServerFound");
		break;
	case NatNetTfBroadcaster::NewRigidBody:
		RCLCPP_DEBUG(get_logger(), "NatNetTfBroadcaster::SetState: NewRigidBody");
		break;
	case NatNetTfBroadcaster::NormalOperation:
		RCLCPP_INFO(get_logger(), "NatNetTfBroadcaster::SetState: NormalOperation");
		break;
	default:
		RCLCPP_WARN(get_logger(), "NatNetTfBroadcaster::SetState: Unknown");
		break;
	}
}

//-----------------------------------------------------------------------------------------------------

void NatNetTfBroadcaster::SetConnectParams(const sNatNetClientConnectParams &natNetClientConnectParams)
{
	m_sConnectParams.serverCommandPort = natNetClientConnectParams.serverCommandPort;
	m_sConnectParams.serverDataPort = natNetClientConnectParams.serverDataPort;
	m_sConnectParams.serverAddress = natNetClientConnectParams.serverAddress;
	m_sConnectParams.localAddress = natNetClientConnectParams.localAddress;
}

//-----------------------------------------------------------------------------------------------------

void NatNetTfBroadcaster::Connect()
{
    // Init Client and connect to NatNet server
	ErrorCode errCode = m_pNatNetClient->Connect(m_sConnectParams);
	if (ErrorCode_OK == errCode)
	{
		errCode = GetServerDescription();
		if(ErrorCode_OK == errCode)
		{
			LogServerDescription();
		}
		else
		{
			RCLCPP_ERROR(get_logger(), "NatNetTfBroadcaster::ServerDiscoveredCallback: couldn't connect to server.");
		}
	}
	else
	{
		RCLCPP_ERROR(get_logger(), "Unable to connect to server.  Error code: %d. Exiting.\n", errCode);
	}

	SetFrameCallbackHandler();
}

//-----------------------------------------------------------------------------------------------------

ErrorCode NatNetTfBroadcaster::GetServerDescription()
{
	return m_pNatNetClient->GetServerDescription(m_pServerDescription);
}

//-----------------------------------------------------------------------------------------------------

void NatNetTfBroadcaster::GetDataDescriptions()
{
	int i;
	ErrorCode eResult;
	sRigidBodyDescription* pRB;

	if(NULL != m_pDataDefs)
	{
		NatNet_FreeDescriptions(m_pDataDefs);
		m_pDataDefs = NULL;
	}

	eResult = m_pNatNetClient->GetDataDescriptionList(&m_pDataDefs);

	if (ErrorCode_OK == eResult && NULL != m_pDataDefs)
	{
		RCLCPP_INFO(get_logger(), "NatNetTfBroadcaster::GetDataDescriptions: %d Data Descriptions:\n",
				m_pDataDefs->nDataDescriptions
				);
		for(i=0; i<m_pDataDefs->nDataDescriptions; i++)
		{
            if(m_pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
            	pRB = m_pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
            	m_mapIdIndex[pRB->ID] = i;
            	RCLCPP_INFO(get_logger(), "NatNetTfBroadcaster::GetDataDescriptions: Object %d:\n"
            			"Name: %s\n"
            			"ID: %d\n"
            			"Parent ID : %d\n"
            			"Parent Offset : %3.2f,%3.2f,%3.2f\n",
						i, pRB->szName, pRB->ID, pRB->parentID, pRB->offsetx, pRB->offsety, pRB->offsetz
						);
            }
        }

	}
	else
	{
		RCLCPP_ERROR(get_logger(), "NatNetTfBroadcaster::GetDataDescriptions: Bad result (%d)!", eResult);
	}
}

//-----------------------------------------------------------------------------------------------------

void NatNetTfBroadcaster::LogServerDescription()
{
	if(m_pServerDescription->HostPresent)
	{
		RCLCPP_INFO(this->get_logger(), "NatNetTfBroadcaster::ServerDescription:\n\
					Server Application: %s (ver. %d.%d.%d.%d)\n\
					NatNet Version: %d.%d.%d.%d\n\
					Host Address: %d.%d.%d.%d\n",
					m_pServerDescription->szHostApp,
					m_pServerDescription->HostAppVersion[0],
					m_pServerDescription->HostAppVersion[1],
					m_pServerDescription->HostAppVersion[2],
					m_pServerDescription->HostAppVersion[3],
					m_pServerDescription->NatNetVersion[0],
					m_pServerDescription->NatNetVersion[1],
					m_pServerDescription->NatNetVersion[2],
					m_pServerDescription->NatNetVersion[3],
					m_pServerDescription->HostComputerAddress[0],
					m_pServerDescription->HostComputerAddress[1],
					m_pServerDescription->HostComputerAddress[2],
					m_pServerDescription->HostComputerAddress[3]);
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "NatNetTfBroadcaster::LogServerDescription: host not present.");
	}
}

//-----------------------------------------------------------------------------------------------------

void NatNetTfBroadcaster::SendRootStaticTransform()
{
    rclcpp::Parameter param_ParentFrameName = this->get_parameter("parent_frame");
    rclcpp::Parameter param_RootFrameName = this->get_parameter("root_frame");
    rclcpp::Parameter param_Location = this->get_parameter("location");
    rclcpp::Parameter param_Orientation = this->get_parameter("orientation");

    this->SetRootFrame(param_RootFrameName.as_string());

    std::vector<double> t = param_Location.as_double_array();
    std::vector<double> q = param_Orientation.as_double_array();

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = param_ParentFrameName.as_string();
    tf_msg.child_frame_id = this->GetRootFrameName();
    tf_msg.transform.translation.x = t[0];
    tf_msg.transform.translation.y = t[1];
    tf_msg.transform.translation.z = t[2];
    tf_msg.transform.rotation.x = q[0];
    tf_msg.transform.rotation.y = q[1];
    tf_msg.transform.rotation.z = q[2];
    tf_msg.transform.rotation.w = q[3];

    this->SendStaticTransform(tf_msg);
}

//-----------------------------------------------------------------------------------------------------

sRigidBodyDescription* NatNetTfBroadcaster::GetRigidBodyDescription(int nId)
{
	sRigidBodyDescription* pResult = NULL;
	if (m_mapIdIndex.end() == m_mapIdIndex.find(nId))
	{
		// nId not found
		RCLCPP_DEBUG(this->get_logger(), "NatNetTfBroadcaster::GetRigidBodyDescription: unrecognized object ID: %d.", nId);
	}
	else
	{
		pResult = m_pDataDefs->arrDataDescriptions[m_mapIdIndex[nId]].Data.RigidBodyDescription;
	}
	return pResult;
}


//-----------------------------------------------------------------------------------------------------
// 										NATNET API procedures
//-----------------------------------------------------------------------------------------------------

void NATNET_CALLCONV NatNetTfBroadcaster::DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	int i;
	int modelID, markerID;
    //bool bOccluded;     // marker was not visible (occluded) in this frame
    //bool bPCSolved;     // reported position provided by point cloud solve
    //bool bModelSolved;  // reported position provided by model solve
    //bool bHasModel;     // marker has an associated asset in the data stream
    bool bUnlabeled;    // marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
	//bool bActiveMarker; // marker is an actively labeled LED marker
    geometry_msgs::msg::TransformStamped tf_msg;

    bool bNewRigidBody(false);

	//rclcpp::Time timecode(data->Timecode, data->TimecodeSubframe);

	NatNetTfBroadcaster* pNatNetTfBroadcaster((NatNetTfBroadcaster*)pUserData);

	/*RCLCPP_INFO(pNatNetTfBroadcaster->get_logger(), "NatNetTfBroadcaster::DataHandler:\n"
			"Timecode: %d.%d\n"
			"Incoming frame number: %d\n"
			"Rigid Bodies [Count=%d]\n"
			"Skeletons [Count=%d]\n"
			"Markers [Count=%d]\n", data->Timecode, data->TimecodeSubframe,
			data->iFrame, data->nRigidBodies, data->nSkeletons, data->nLabeledMarkers);
	*/

	tf_msg.header.stamp = pNatNetTfBroadcaster->get_clock()->now();

	rclcpp::Parameter param_RootOrientation = pNatNetTfBroadcaster->get_parameter("orientation");

	std::vector<double> q = param_RootOrientation.as_double_array();

	// Rigid Bodies
	for(i=0; (false == bNewRigidBody) && (i < data->nRigidBodies); i++)
	{
		if(data->RigidBodies[i].params & 0x01)
		{
        	sRigidBodyDescription* pRB = pNatNetTfBroadcaster->GetRigidBodyDescription(data->RigidBodies[i].ID);

        	if (NULL == pRB)
        	{
        		bNewRigidBody = true;
        	}
        	else
        	{
        		if(-1 == pRB->parentID)
        		{
        			tf_msg.header.frame_id = pNatNetTfBroadcaster->GetRootFrameName();
        		}
        		else
        		{
        			tf_msg.header.frame_id = pNatNetTfBroadcaster->GetRigidBodyDescription(pRB->parentID)->szName;
        		}

        		tf_msg.header.frame_id = pNatNetTfBroadcaster->GetRootFrameName();
        		tf_msg.child_frame_id = tf_msg.header.frame_id + "_" + pRB->szName;
        		tf_msg.transform.translation.x = data->RigidBodies[i].x;
        		tf_msg.transform.translation.y = data->RigidBodies[i].y;
        		tf_msg.transform.translation.z = data->RigidBodies[i].z;
        		tf_msg.transform.rotation.x = data->RigidBodies[i].qx;
        		tf_msg.transform.rotation.y = data->RigidBodies[i].qy;
        		tf_msg.transform.rotation.z = data->RigidBodies[i].qz;
        		tf_msg.transform.rotation.w = data->RigidBodies[i].qw;

        		/*RCLCPP_INFO(pNatNetTfBroadcaster->get_logger(), "NatNetTfBroadcaster::DataHandler: RigidBodies\n"
        			"tf_msg.header.stamp: %d.%d\n"
					"tf_msg.header.frame_id: %s\n"
					"tf_msg.child_frame_id: %s\n"
					"tf_msg.transform.translation.x %f\n"
					"tf_msg.transform.translation.y %f\n"
					"tf_msg.transform.translation.z %f\n"
					"tf_msg.transform.rotation.x =  %f\n"
					"tf_msg.transform.rotation.y =  %f\n"
					"tf_msg.transform.rotation.z =  %f\n"
					"tf_msg.transform.rotation.w =  %f\n",
					tf_msg.header.stamp.sec, tf_msg.header.stamp.nanosec,
					tf_msg.header.frame_id.c_str(),
					tf_msg.child_frame_id.c_str(),
					tf_msg.transform.translation.x,
					tf_msg.transform.translation.y,
					tf_msg.transform.translation.z,
					tf_msg.transform.rotation.x,
					tf_msg.transform.rotation.y,
					tf_msg.transform.rotation.z,
					tf_msg.transform.rotation.w
					);*/

        		pNatNetTfBroadcaster->SendTransform(tf_msg);

        		// From the mocap coordinate system to the parent coordinate system
        		tf_msg.header.frame_id = tf_msg.child_frame_id;
        		tf_msg.child_frame_id = pRB->szName;
        		tf_msg.transform.translation.x = 0;
        		tf_msg.transform.translation.y = 0;
        		tf_msg.transform.translation.z = 0;
        		tf_msg.transform.rotation.x = -q[0];
        		tf_msg.transform.rotation.y = -q[1];
        		tf_msg.transform.rotation.z = -q[2];
        		tf_msg.transform.rotation.w = q[3];

        		pNatNetTfBroadcaster->SendStaticTransform(tf_msg);
        	}
		}
	}

	if (bNewRigidBody)
	{
		pNatNetTfBroadcaster->SetState(NatNetTfBroadcaster::NewRigidBody);
	}

	// labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
	for(i=0; i < data->nLabeledMarkers; i++)
	{
        //bOccluded = ((data->LabeledMarkers[i].params & 0x01)!=0);
        //bPCSolved = ((data->LabeledMarkers[i].params & 0x02)!=0);
        //bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
        //bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
        bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
		//bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

        sMarker marker = data->LabeledMarkers[i];

        NatNet_DecodeID(marker.ID, &modelID, &markerID);

        if(bUnlabeled)
        {
        	tf_msg.header.frame_id = pNatNetTfBroadcaster->GetRootFrameName();
        	tf_msg.child_frame_id = "Unlabeled_" + std::to_string(markerID);
        	tf_msg.transform.translation.x = marker.x;
        	tf_msg.transform.translation.y = marker.y;
        	tf_msg.transform.translation.z = marker.z;
        	tf_msg.transform.rotation.x = 0;
        	tf_msg.transform.rotation.y = 0;
        	tf_msg.transform.rotation.z = 0;
        	tf_msg.transform.rotation.w = 1;

        	pNatNetTfBroadcaster->SendTransform(tf_msg);
        }
	}
}

//-----------------------------------------------------------------------------------------------------

void NATNET_CALLCONV NatNetTfBroadcaster::ServerDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext)
{
	NatNetTfBroadcaster* pNatNetTfBroadcaster((NatNetTfBroadcaster*)pUserContext);

	if (pDiscoveredServer->serverDescription.ConnectionMulticast)
	{
		std::string errMessage("NatNetTfBroadcaster::ServerDiscoveredCallback: Multicast currently unsupported");
		RCLCPP_ERROR(pNatNetTfBroadcaster->get_logger(), errMessage.c_str());
		throw std::runtime_error(errMessage);
	}

	if ( pDiscoveredServer->serverDescription.bConnectionInfoValid)
	{
		// Build the connection parameters.
		sNatNetClientConnectParams sConnectParams;
		sConnectParams.serverCommandPort = pDiscoveredServer->serverCommandPort;
		sConnectParams.serverDataPort = pDiscoveredServer->serverDescription.ConnectionDataPort;
		sConnectParams.serverAddress = pDiscoveredServer->serverAddress;
		sConnectParams.localAddress = pDiscoveredServer->localAddress;

		pNatNetTfBroadcaster->SetConnectParams(sConnectParams);
	}
	else
	{
		std::string errMessage("NatNetTfBroadcaster::ServerDiscoveredCallback: Legacy server, could not autodetect settings.");
		RCLCPP_ERROR(pNatNetTfBroadcaster->get_logger(), errMessage.c_str());
		throw std::runtime_error(errMessage);
	}

	pNatNetTfBroadcaster->SetState(NatNetTfBroadcaster::ServerFound);
}

//-----------------------------------------------------------------------------------------------------

} // namespace mocap
