// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file cadenaParticipant.cpp
 * This file contains the implementation of the participant functions.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/Domain.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>

#include <thread>
#include <chrono>
#include "dsrparticipant.h"

//using namespace eprosima::fastrtps;
//using namespace eprosima::fastrtps::rtps;

DSRParticipant::DSRParticipant() : mp_publisher(nullptr), mp_subscriber(nullptr) {}

DSRParticipant::~DSRParticipant() 
{ 
    eprosima::fastrtps::Domain::removeParticipant(mp_participant);
}

std::tuple<bool, eprosima::fastrtps::Participant *> DSRParticipant::init()
{
    // Create RTPSParticipant     
    eprosima::fastrtps::ParticipantAttributes PParam;
    PParam.rtps.setName("Participant_Participant");  //You can put here the name you want
    
    //Create a descriptor for the new transport.
    auto custom_transport = std::make_shared<eprosima::fastrtps::rtps::UDPv4TransportDescriptor>();
    custom_transport->sendBufferSize = 12582912; //33554432;
    custom_transport->receiveBufferSize = 12582912; //33554432;
    custom_transport->maxMessageSize = 65000;
    custom_transport->interfaceWhiteList.emplace_back("127.0.0.1");
    //custom_transport->interfaceWhiteList.emplace_back("192.168.1.253");


    //Disable the built-in Transport Layer.
    PParam.rtps.useBuiltinTransports = false;

    //Link the Transport Layer to the Participant.
    PParam.rtps.userTransports.push_back(custom_transport);

    mp_participant = eprosima::fastrtps::Domain::createParticipant(PParam);

    if(mp_participant == nullptr)
    {
        return std::make_tuple(false, nullptr);
    }

    //Register the type
    eprosima::fastrtps::Domain::registerType(mp_participant, static_cast<eprosima::fastrtps::TopicDataType*>(&dsrgraphType));
    eprosima::fastrtps::Domain::registerType(mp_participant, static_cast<eprosima::fastrtps::TopicDataType*>(&graphrequestType));
    eprosima::fastrtps::Domain::registerType(mp_participant, static_cast<eprosima::fastrtps::TopicDataType*>(&graphRequestAnswerType));

    return std::make_tuple(true, mp_participant);
}

eprosima::fastrtps::Participant* DSRParticipant::getParticipant() 
{
    return mp_participant;
}

eprosima::fastrtps::rtps::GUID_t DSRParticipant::getID() const
{   
    return mp_participant->getGuid();
}

const char * DSRParticipant::getDSRTopicName() const
{   
    return dsrgraphType.getName();
}

const char * DSRParticipant::getRequestTopicName() const
{   
    return graphrequestType.getName();
}

const char * DSRParticipant::getAnswerTopicName() const
{   
    return graphRequestAnswerType.getName();
}
