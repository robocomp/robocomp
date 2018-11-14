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

#ifndef TOUCHSENSORSERVER_H
#define TOUCHSENSORSERVER_H

#include "touchsensorI.h"

class TouchSensorServer
{
	public:
		TouchSensorServer(Ice::CommunicatorPtr communicator, 	std::shared_ptr<SpecificWorker> worker_, uint32_t _port);
		void add(InnerModelTouchSensor *sensor);
		void remove(InnerModelTouchSensor *sensor);
		bool empty();
		void shutdown();

		uint32_t port;
		Ice::CommunicatorPtr comm;
		Ice::ObjectAdapterPtr adapter;
		TouchSensorI *interface;
		std::vector<InnerModelTouchSensor *> sensors;
		std::shared_ptr<SpecificWorker> worker;
};

#endif // TOUCHSENSORSERVER_H
