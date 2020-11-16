#ifndef _PARTICIPANT_H_
#define _PARTICIPANT_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/log/Log.h>
#include <fastdds/rtps/RTPSDomain.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>

#include "../topics/IDLGraphPubSubTypes.h"
#include "./dsrpublisher.h"
#include "./dsrsubscriber.h"

class DSRParticipant
{
public:
    DSRParticipant();
    virtual ~DSRParticipant();
    std::tuple<bool, eprosima::fastdds::dds::DomainParticipant *> init(int32_t agent_id, int localhost);
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
    void remove_participant();
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


};

#endif // _Participant_H_