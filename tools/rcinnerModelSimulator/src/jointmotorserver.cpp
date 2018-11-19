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

#include "jointmotorserver.h"

JointMotorServer::JointMotorServer(Ice::CommunicatorPtr communicator, std::shared_ptr<SpecificWorker> worker_, uint32_t _port)
{
	port = _port;
	worker = worker_;
	std::stringstream out1;
	out1 << port;
	comm = communicator;
	std::string name = std::string("JointMotor") + out1.str();
	std::string endp = std::string("tcp -p ")    + out1.str();

	adapter = communicator->createObjectAdapterWithEndpoints(name, endp);
	printf("Creating JointMotor adapter <<%s>> with endpoint <<%s>>\n", name.c_str(), endp.c_str());
	interface = new JointMotorI(worker);
	adapter->add(interface, Ice::stringToIdentity("jointmotor"));
	adapter->activate();
}


void JointMotorServer::add(InnerModelJoint *joint)
{
	joints.push_back(joint);
	interface->add(joint->id);
	joint->setAngle(joint->home);
}

void JointMotorServer::add(InnerModelPrismaticJoint *joint)
{
	joints.push_back(joint);
	interface->add(joint->id);
}

void JointMotorServer::remove(InnerModelJoint *joint)
{
	interface->remove(joint->id);
	joints.erase(std::remove(joints.begin(), joints.end(), joint), joints.end());
}

void JointMotorServer::remove(InnerModelPrismaticJoint *joint)
{
	interface->remove(joint->id);
	joints.erase(std::remove(joints.begin(), joints.end(), joint), joints.end());
}

bool JointMotorServer::empty()
{
	if (joints.size()==0)
		return true;
	return false;
}

void JointMotorServer::shutdown()
{
	try
	{
		adapter->remove(Ice::stringToIdentity("jointmotor"));
	}
	catch(Ice::ObjectAdapterDeactivatedException e)
	{
	}

	adapter->destroy();
}
