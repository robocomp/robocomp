/*
 * Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "serversinitiator.h"

void ServersInitiator::init(std::shared_ptr<InnerModel> innerModel_, std::shared_ptr<InnerModelViewer> imv_ , 
														std::shared_ptr<SpecificWorker> worker_, Ice::CommunicatorPtr communicator_)
{
	innerModel = innerModel_;
	imv = imv_;
	worker = worker_;
	communicator = communicator_;
	walkTree();
	for (auto &it :imv->lasers)
		addServer<InnerModelNode, InnerModelLaser, LaserServer>(it.laserNode);
	for (auto &it : imv->cameras)
		addServer<InnerModelNode, InnerModelRGBD, RGBDServer>(it.RGBDNode);
	
 	//auto print_container = [](andyg::HeteroContainer& _in){_in.visit(printVisitor{}); std::cout << std::endl;};
	//print_container(hMaps);
}

void ServersInitiator::walkTree(InnerModelNode *node)
{
	if (node == nullptr)
		node = innerModel->getRoot();
	
	for(auto &it : node->children)	
	{
		addServer<InnerModelNode, InnerModelDifferentialRobot, DifferentialRobotServer>(it);
		addServer<InnerModelNode, InnerModelOmniRobot, OmniRobotServer>(it);
		addServer<InnerModelNode, InnerModelDisplay, DisplayServer>(it);
		addServer<InnerModelNode, InnerModelTouchSensor, TouchSensorServer>(it);
		addServer<InnerModelNode, InnerModelIMU, IMUServer>(it);
		addServer<InnerModelNode, InnerModelJoint, JointMotorServer>(it);
		addServer<InnerModelNode, InnerModelPrismaticJoint, JointMotorServer>(it);
		walkTree(it);
	}
}

void ServersInitiator::removeJointMotorServer(InnerModelJoint *node)
{
	for (auto &[k, v] : hMaps.getMap<JointMotorServer>())
	{
		v.remove(node);
		if (v.empty())
		{
			jointServersToShutDown.push_back(&(v));
			hMaps.erase<JointMotorServer>(k);
		}
	}
}

void ServersInitiator::shutdownEmptyServers()
{
	for(auto &js : jointServersToShutDown)
			js->shutdown();
	jointServersToShutDown.clear();
}
