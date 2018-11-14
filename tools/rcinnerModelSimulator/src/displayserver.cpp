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

#include "displayserver.h"

DisplayServer::DisplayServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker_, uint32_t _port)
{
	port = _port;
	worker = worker_;
	std::stringstream out1;
	out1 << port;
	comm = communicator;
	std::string name = std::string("Display") + out1.str();
	std::string endp = std::string("tcp -p ")    + out1.str();

	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);
	printf("Creating Display adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new DisplayI(worker);
	adapter->add(interface, Ice::stringToIdentity("display"));
	adapter->activate();
}

void DisplayServer::add(InnerModelDisplay *display)
{
	interface->add(display->id);
}

bool DisplayServer::empty()
{
	return false;
}

void DisplayServer::shutdown()
{
	try
	{
		adapter->remove(Ice::stringToIdentity("display"));
	}
	catch(Ice::ObjectAdapterDeactivatedException e)
	{
	}

	adapter->destroy();
}
