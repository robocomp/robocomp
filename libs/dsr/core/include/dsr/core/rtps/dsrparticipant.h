#ifndef _PARTICIPANT_H_
#define _PARTICIPANT_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/log/Log.h>
#include <fastdds/rtps/RTPSDomain.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>

#include <dsr/core/topics/IDLGraphPubSubTypes.h>
#include <dsr/core/rtps/dsrpublisher.h>
#include <dsr/core/rtps/dsrsubscriber.h>

class DSRParticipant
{
public:
    DSRParticipant();
    virtual ~DSRParticipant();
    [[nodiscard]] std::tuple<bool, eprosima::fastdds::dds::DomainParticipant *> init(uint32_t agent_id, int localhost, std::function<void(eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&&)> fn);
    [[nodiscard]] const eprosima::fastrtps::rtps::GUID_t& getID() const;
    [[nodiscard]] const char *getNodeTopicName()     const { return dsrgraphType->getName();}
    [[nodiscard]] const char *getRequestTopicName()  const { return graphrequestType->getName();}
    [[nodiscard]] const char *getAnswerTopicName()   const { return graphRequestAnswerType->getName();}
    [[nodiscard]] const char *getEdgeTopicName()     const { return dsrEdgeType->getName();}
    [[nodiscard]] const char *getNodeAttrTopicName() const { return dsrNodeAttrType->getName();}
    [[nodiscard]] const char *getEdgeAttrTopicName() const { return dsrEdgeAttrType->getName();}

    [[nodiscard]] eprosima::fastdds::dds::Topic*  getNodeTopic()          { return topic_node; }
    [[nodiscard]] eprosima::fastdds::dds::Topic*  getEdgeTopic()          { return topic_edge; }
    [[nodiscard]] eprosima::fastdds::dds::Topic*  getGraphTopic()         { return topic_graph;}
    [[nodiscard]] eprosima::fastdds::dds::Topic*  getGraphRequestTopic()  { return topic_graph_request;}
    [[nodiscard]] eprosima::fastdds::dds::Topic*  getAttNodeTopic()       { return topic_node_att;}
    [[nodiscard]] eprosima::fastdds::dds::Topic*  getAttEdgeTopic()       { return topic_edge_att;}
    [[nodiscard]] eprosima::fastdds::dds::DomainParticipant *getParticipant();

    void add_subscriber(const std::string& id, std::pair<eprosima::fastdds::dds::Subscriber*, eprosima::fastdds::dds::DataReader*>);
    void add_publisher(const std::string& id, std::pair<eprosima::fastdds::dds::Publisher*, eprosima::fastdds::dds::DataWriter*>);
    void delete_subscriber(const std::string& id);
    void delete_publisher(const std::string& id);

    void remove_participant_and_entities();

private:
    eprosima::fastdds::dds::DomainParticipant* mp_participant{};

    eprosima::fastdds::dds::Topic*  topic_node{};
    eprosima::fastdds::dds::Topic*  topic_edge{};
    eprosima::fastdds::dds::Topic*  topic_graph{};
    eprosima::fastdds::dds::Topic*  topic_graph_request{};
    eprosima::fastdds::dds::Topic*  topic_node_att{};
    eprosima::fastdds::dds::Topic*  topic_edge_att{};

    eprosima::fastdds::dds::TypeSupport dsrgraphType{};
    eprosima::fastdds::dds::TypeSupport graphrequestType{};
    eprosima::fastdds::dds::TypeSupport graphRequestAnswerType{};
    eprosima::fastdds::dds::TypeSupport dsrEdgeType{};
    eprosima::fastdds::dds::TypeSupport dsrNodeAttrType{};
    eprosima::fastdds::dds::TypeSupport dsrEdgeAttrType{};

    std::map<std::string, std::pair<eprosima::fastdds::dds::Subscriber*, eprosima::fastdds::dds::DataReader*>> subscribers;
    std::map<std::string, std::pair<eprosima::fastdds::dds::Publisher*, eprosima::fastdds::dds::DataWriter*>> publishers;
    mutable std::mutex pub_mtx;
    mutable std::mutex sub_mtx;

    class ParticpantListener : public eprosima::fastdds::dds::DomainParticipantListener
    {
    public:
        explicit ParticpantListener(std::function<void(eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&&)>&& fn)
            : eprosima::fastdds::dds::DomainParticipantListener(), f(std::move(fn)){};
        ~ParticpantListener() override = default;

         void on_participant_discovery  (
                eprosima::fastdds::dds::DomainParticipant* participant,
                eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&& info) override
        {
            /*if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT)
            {
                //Nothing to do here at the moment.
            }
            else if (info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT ||
                     info.status == eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT)
            {*/
            //Callback
            f(std::forward<eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&&>(info));
            /*}*/
        }


        std::function<void(eprosima::fastrtps::rtps::ParticipantDiscoveryInfo&&)> f;
        //int n_matched;
    };
    std::unique_ptr<ParticpantListener> m_listener;
};

#endif // _Participant_H_