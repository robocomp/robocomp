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

#include "rgbdserver.h"

RGBDServer::RGBDServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker, uint32_t _port)
{
	port = _port;
	std::stringstream out1;
	out1 << port;
	std::string name = std::string("RGBD") + out1.str();
	std::string endp = std::string("tcp -p ") + out1.str();
	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);
	printf("Creating RGBD adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new RGBDI(worker);
	adapter->add(interface, Ice::stringToIdentity("rgbd"));
	adapter->activate();
}

void RGBDServer::add(InnerModelRGBD *rgbd)
{
	rgbds.push_back(rgbd);
	interface->add(rgbd->id);
}

