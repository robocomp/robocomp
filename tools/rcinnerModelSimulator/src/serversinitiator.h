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

#ifndef SERVERSINITIATOR_H
#define SERVERSINITIATOR_H

#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>
#include "servers.h"
#include "omnirobotserver.h"
#include "differentialrobotserver.h"
#include "heterocontainer.h"
#include "displayserver.h"
#include "rgbdserver.h"
#include "laserserver.h"
#include "touchsensorserver.h"
#include "imuserver.h"
#include "jointmotorserver.h"

class ServersInitiator
{
	public:
		void init(std::shared_ptr<InnerModel> inner, std::shared_ptr<InnerModelViewer> imv_ , 
							std::shared_ptr<SpecificWorker> worker_,  Ice::CommunicatorPtr communicator_);
		void shutdownEmptyServers();
		void removeJointMotorServer(InnerModelJoint *node);
		void walkTree(InnerModelNode *node = nullptr);
		
		QList <JointMotorServer *> jointServersToShutDown;
		
		template<typename TA, typename TN, typename TS>
		void addServer(TA *s)
		{
			auto *node = dynamic_cast<TN*>(s);
			if(node != nullptr)
			{
				std::uint32_t &port = node->port;
				hMaps.insert(port, TS(communicator, worker, port));
				hMaps.at<TS>(port).add(node);
			}
		}
		
		andyg::HeteroContainer hMaps;
		
		Ice::CommunicatorPtr communicator;
		std::shared_ptr<InnerModel> innerModel;
		std::shared_ptr<SpecificWorker> worker;
		std::shared_ptr<InnerModelViewer> imv;
};

#endif // SERVERSINITIATOR_H


	/*struct printVisitor : andyg::visitor_base<DifferentialRobotServer, OmniRobotServer>
		{
			template<class T>
			void operator()(T& _in)
			{
				std::cout << _in.name << " ";
			}
		};
	*/	
