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
                                   dsrEdgeAttrType(new MvregEdgeAttrVecPubSubType()),
                                   m_listener(nullptr)

{}

DSRParticipant::~DSRParticipant()
{

    if (mp_participant != nullptr)
    {
        {
            std::unique_lock<std::mutex> lck (sub_mtx);
            auto it = subscribers.begin();
            while (it != subscribers.end()) {
                auto[sub, reader] = it->second;
                if (mp_participant != nullptr) {
                    if (reader != nullptr && sub != nullptr) {
                        [[maybe_unused]] auto res = sub->delete_datareader(reader);
                    }
                    if (sub != nullptr) {
                        [[maybe_unused]] auto res = mp_participant->delete_subscriber(sub);
                    }
                }
                it = subscribers.erase(it);
            }
        }
        {
            std::unique_lock<std::mutex> lck (pub_mtx);
            auto it2 = publishers.begin();
            while (it2 != publishers.end()) {
                auto[pub, writer] = it2->second;
                if (mp_participant != nullptr) {
                    if (writer != nullptr && pub != nullptr) {
                        [[maybe_unused]] auto res = pub->delete_datawriter(writer);
                    }
                    if (pub != nullptr) {
                        [[maybe_unused]] auto res = mp_participant->delete_publisher(pub);
                    }
                }
                it2 = publishers.erase(it2);
            }
        }

        if (topic_node)
        {
            if(mp_participant->delete_topic(topic_node) == ReturnCode_t::RETCODE_PRECONDITION_NOT_MET)
            {
                std::cout << " Remove topic error " << topic_node->get_name() << std::endl;
            }
        }
        if (topic_edge)
        {
            if(mp_participant->delete_topic(topic_edge) == ReturnCode_t::RETCODE_PRECONDITION_NOT_MET)
            {
                std::cout << " Remove topic error " << topic_edge->get_name() << std::endl;
            }
        }
        if (topic_graph)
        {
            topic_graph->close();
            if(mp_participant->delete_topic(topic_graph) == ReturnCode_t::RETCODE_PRECONDITION_NOT_MET)
            {
                std::cout << " Remove topic error " << topic_graph->get_name() << std::endl;
            }
        }
        if (topic_graph_request)
        {
            topic_graph_request->close();
            if(mp_participant->delete_topic(topic_graph_request) == ReturnCode_t::RETCODE_PRECONDITION_NOT_MET)
            {
                std::cout << " Remove topic error " << topic_graph_request->get_name() << std::endl;
            }
        }
        if (topic_node_att)
        {
            topic_node_att->close();
            if(mp_participant->delete_topic(topic_node_att) == ReturnCode_t::RETCODE_PRECONDITION_NOT_MET)
            {
                std::cout << " Remove topic error " << topic_node_att->get_name() << std::endl;
            }
        }
        if (topic_edge_att)
        {
            topic_edge_att->close();
            if(mp_participant->delete_topic(topic_edge_att) == ReturnCode_t::RETCODE_PRECONDITION_NOT_MET)
            {
                std::cout << " Remove topic error " << topic_edge_att->get_name() << std::endl;
            }

        }

        auto res = eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(mp_participant);
        if (res == ReturnCode_t::RETCODE_PRECONDITION_NOT_MET) {
            std::cout << "Error removing participant. There are entities in use." <<std::endl;
        }

    }

    qDebug()  << "Removing DSRParticipant" ;

}

std::tuple<bool, eprosima::fastdds::dds::DomainParticipant*> DSRParticipant::init(int32_t agent_id, int localhost, std::function<void(eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&&)> fn)
{
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

        custom_transport->interfaceWhiteList.emplace_back("127.0.0.1");

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

    m_listener = std::make_unique<ParticpantListener>(std::move(fn));

    int retry = 0;
    while (retry < 5) {
        mp_participant = DomainParticipantFactory::get_instance()->create_participant(0, PParam, m_listener.get(), StatusMask::none());
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
    topic_node = mp_participant->create_topic("DSR_NODE", dsrgraphType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_edge = mp_participant->create_topic("DSR_EDGE", dsrEdgeType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_node_att = mp_participant->create_topic("DSR_NODE_ATTS", dsrNodeAttrType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_edge_att = mp_participant->create_topic("DSR_EDGE_ATTS", dsrEdgeAttrType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_graph_request = mp_participant->create_topic("GRAPH_REQUEST", graphrequestType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    topic_graph = mp_participant->create_topic("GRAPH_ANSWER", graphRequestAnswerType.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);

    return std::make_tuple(true, mp_participant);
}

eprosima::fastdds::dds::DomainParticipant *DSRParticipant::getParticipant()
{
    return mp_participant;
}

void DSRParticipant::remove_participant()
{
    if (mp_participant != nullptr) {
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(mp_participant);
        mp_participant = nullptr;
    }
}

const eprosima::fastrtps::rtps::GUID_t& DSRParticipant::getID() const
{
    return mp_participant->guid();
}

void DSRParticipant::add_subscriber(const std::string& id, std::pair<eprosima::fastdds::dds::Subscriber*, eprosima::fastdds::dds::DataReader*> val)
{
    std::unique_lock<std::mutex> lck (sub_mtx);
    subscribers.emplace(id, val);
}
void DSRParticipant::add_publisher(const std::string& id, std::pair<eprosima::fastdds::dds::Publisher*, eprosima::fastdds::dds::DataWriter*> val)
{
    std::unique_lock<std::mutex> lck (pub_mtx);
    publishers.emplace(id, val);
}

void DSRParticipant::disable_subscriber(const std::string& id)
{
    std::unique_lock<std::mutex> lck (sub_mtx);
    try {
        auto[sub, reader] = subscribers.at(id);
        if (mp_participant != nullptr)
        {
            if (reader != nullptr && sub != nullptr)
            {
                //reader->close();
                [[maybe_unused]] auto res = sub->delete_datareader(reader);
                //std::cout << " Remove reader " << id << " " << res() << std::endl;

            }

            if (sub != nullptr)
            {
                //sub->close();
                [[maybe_unused]] auto res = mp_participant->delete_subscriber(sub);
                //std::cout << " Remove sub " << id << " " << res() << std::endl;

            }
        }
        subscribers.erase(id);
    } catch (...) {
        std::cout << "delete sub error: " << id << std::endl;
    }
}

void DSRParticipant::disable_publisher(const std::string& id)
{
    std::unique_lock<std::mutex> lck (pub_mtx);
    try {
        auto[pub, writer] = publishers.at(id);
        if (mp_participant != nullptr)
        {
            if (writer != nullptr && pub != nullptr)
            {
                //writer->close();
                [[maybe_unused]] auto res = pub->delete_datawriter(writer);
                //std::cout << " Remove writer " << id << " " << res() << std::endl;

            }

            if (pub != nullptr)
            {
                //pub->close();
                [[maybe_unused]] auto res = mp_participant->delete_publisher(pub);
                //std::cout << " Remove pub " << id << " " << res() << std::endl;
            }
        }
        publishers.erase(id);
    } catch (...) {
        std::cout << "delete pub error: " << id << std::endl;
    }
}