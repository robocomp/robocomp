#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>

#include <fastrtps/utils/IPFinder.h>

#include <thread>
#include <QDebug>
#include "dsrparticipant.h"

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;

DSRParticipant::DSRParticipant() : mp_participant(nullptr),
                                   dsrgraphType(new MvregNodePubSubType()),
                                   graphrequestType(new GraphRequestPubSubType()),
                                   graphRequestAnswerType(new OrMapPubSubType()),
                                   dsrEdgeType(new MvregEdgePubSubType()),
                                   dsrNodeAttrType(new MvregNodeAttrVecPubSubType()),
                                   dsrEdgeAttrType(new MvregEdgeAttrVecPubSubType())

{}

DSRParticipant::~DSRParticipant()
{
	
    if (mp_participant != nullptr)
    {
        if (topic_node)
            mp_participant->delete_topic(topic_node);
        if (topic_edge)
            mp_participant->delete_topic(topic_edge);
        if (topic_graph)
            mp_participant->delete_topic(topic_graph);
        if (topic_graph_request)
            mp_participant->delete_topic(topic_graph_request);
        if (topic_node_att)
            mp_participant->delete_topic(topic_node_att);
        if (topic_edge_att)
            mp_participant->delete_topic(topic_edge_att);

        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(mp_participant);
    }
}

std::tuple<bool, eprosima::fastdds::dds::DomainParticipant*> DSRParticipant::init(int32_t agent_id, int localhost) {
    // Create RTPSParticipant     
    DomainParticipantQos PParam;
    PParam.name(("Participant_" + std::to_string(agent_id)).data());  //You can put here the name you want


    //Disable the built-in Transport Layer.
    PParam.transport().use_builtin_transports = false;

    //Create a descriptor for the new transport.
    auto custom_transport = std::make_shared<UDPv4TransportDescriptor>();
    custom_transport->sendBufferSize = 33554432;
    custom_transport->receiveBufferSize = 33554432;
    custom_transport->maxMessageSize = 65000;
    PParam.transport().user_transports.push_back(custom_transport);

    if (not localhost)
    {

        std::vector<eprosima::fastrtps::rtps::IPFinder::info_IP> ips;
        eprosima::fastrtps::rtps::IPFinder::getIPs(&ips, false);

        for (auto &ip : ips) {
            if (ip.type == eprosima::fastrtps::rtps::IPFinder::IP4 ) {
                custom_transport->interfaceWhiteList.emplace_back(ip.name);
            }
        }

    } else {
        // Create a descriptor for same device agents.
        auto shm_transport = std::make_shared<SharedMemTransportDescriptor>();
        shm_transport->segment_size(2 * 1024 * 1024);

        PParam.transport().user_transports.push_back(shm_transport);
    }
    PParam.transport().send_socket_buffer_size = 33554432;
    PParam.transport().listen_socket_buffer_size = 33554432;


    //Discovery
    PParam.wire_protocol().builtin.discovery_config.ignoreParticipantFlags =
            static_cast<eprosima::fastrtps::rtps::ParticipantFilteringFlags_t>(
            eprosima::fastrtps::rtps::ParticipantFilteringFlags_t::FILTER_SAME_PROCESS);

    PParam.wire_protocol().builtin.discovery_config.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
    PParam.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod =
            eprosima::fastrtps::Duration_t(3, 0);

    eprosima::fastrtps::Log::SetVerbosity(eprosima::fastdds::dds::Log::Info);

    int retry = 0;
    while (retry < 5) {
        mp_participant = DomainParticipantFactory::get_instance()->create_participant(0, PParam);
        if(mp_participant != nullptr) break;
        retry++;
        qDebug() << "Error creating participant, retrying. [" << retry <<"/5]";
    }

    if(mp_participant == nullptr)
    {
        qFatal("Could not create particpant after 5 attemps");
    }


    //Register types
    dsrgraphType.register_type(mp_participant);
    dsrgraphType.register_type(mp_participant);
    graphrequestType.register_type(mp_participant);
    graphRequestAnswerType.register_type(mp_participant);
    dsrEdgeType.register_type(mp_participant);
    dsrNodeAttrType.register_type(mp_participant);
    dsrEdgeAttrType.register_type(mp_participant);

    //Create topics
    /* Hay que meter esto?
    eprosima::fastrtps::PublisherAttributes Wparam;
    Wparam.topic.topicKind = eprosima::fastrtps::rtps::NO_KEY;
    Wparam.topic.topicDataType = topicDataType;  //This type MUST be registered
    Wparam.topic.topicName = topicName;
    */
    topic_node = mp_participant->create_topic("DSR_NODE", dsrgraphType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_edge = mp_participant->create_topic("DSR_EDGE", dsrEdgeType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_node_att = mp_participant->create_topic("DSR_NODE_ATTS", dsrNodeAttrType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_edge_att = mp_participant->create_topic("DSR_EDGE_ATTS", dsrEdgeAttrType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_graph_request = mp_participant->create_topic("DSR_GRAPH_REQUEST", graphrequestType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_graph = mp_participant->create_topic("DSR_GRAPH_ANSWER", graphRequestAnswerType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

    return std::make_tuple(true, mp_participant);
}

eprosima::fastdds::dds::DomainParticipant *DSRParticipant::getParticipant()
{
    return mp_participant;
}

void DSRParticipant::remove_participant()
{
    if (mp_participant != nullptr)
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(mp_participant);
}

const eprosima::fastrtps::rtps::GUID_t& DSRParticipant::getID() const
{
    return mp_participant->guid();
}

