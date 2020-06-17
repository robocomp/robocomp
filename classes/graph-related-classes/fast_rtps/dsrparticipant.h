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
 * @file particpant.h
 * This header file contains the declaration of the participant functions.
 */


#ifndef _CADENA_PARTICIPANT_H_
#define _CADENA_PARTICIPANT_H_

#include <fastrtps/fastrtps_fwd.h>
#include "../topics/DSRGraphPubSubTypes.h"
#include "dsrpublisher.h"
#include "dsrsubscriber.h"

class DSRParticipant
{
public:
	DSRParticipant();
	virtual ~DSRParticipant();
	std::tuple<bool, eprosima::fastrtps::Participant *> init();
	void run();
	eprosima::fastrtps::rtps::GUID_t getID() const ;
	const char* getDSRTopicName() const;
	const char* getRequestTopicName() const;
	const char* getAnswerTopicName() const;

	eprosima::fastrtps::Participant* getParticipant();

private:
	eprosima::fastrtps::Participant *mp_participant; 
    eprosima::fastrtps::Publisher *mp_publisher; //"DSR"
	eprosima::fastrtps::Subscriber *mp_subscriber; //"DSR"
 	AworSetPubSubType dsrgraphType;

	eprosima::fastrtps::Subscriber *mp_subscriber_graph_request; // "DSR_GRAPH_REQUEST"
	GraphRequestPubSubType graphrequestType;
    
	eprosima::fastrtps::Publisher *mp_publisher_topic_answer; //"DSR_GRAPH_ANSWER"
	eprosima::fastrtps::Subscriber *mp_subscriber_topic_answer; ///"DSR_GRAPH_ANSWER"
	OrMapPubSubType graphRequestAnswerType;


};

#endif // _CADENA_Participant_H_