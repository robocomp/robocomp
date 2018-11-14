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

#ifndef OMNIROBOTSERVER_H
#define OMNIROBOTSERVER_H

#include "omnirobotI.h"
#include "differentialrobotI.h"
#include "genericbaseI.h"

class OmniRobotServer
{
	public:
		OmniRobotServer(){};
		OmniRobotServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker, uint32_t _port);
		void add(InnerModelOmniRobot *omnirobot);

		uint32_t port;
		Ice::ObjectAdapterPtr adapter;
		OmniRobotI *interface;
		DifferentialRobotI *interfaceDFR;
		GenericBaseI *interfaceGB;
		std::vector<InnerModelOmniRobot *> omnirobots;
		std::string name = "OmniRobotServer";
};

#endif // OMNIROBOTSERVER_H
