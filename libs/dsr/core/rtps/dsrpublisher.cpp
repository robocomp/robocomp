#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/Domain.h>
#include <fastrtps/transport/TransportDescriptorInterface.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastrtps/utils/IPFinder.h>

#include <dsr/core/rtps/dsrpublisher.h>

#include <QDebug>


using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

DSRPublisher::DSRPublisher() : mp_participant(nullptr), mp_publisher(nullptr), mp_writer(nullptr)
{}

DSRPublisher::~DSRPublisher()
{
    /*
    if (mp_writer != nullptr && mp_publisher != nullptr)
    {
        mp_publisher->delete_datawriter(mp_writer);
    }

    if (mp_participant != nullptr && mp_publisher != nullptr)
    {
        mp_participant->delete_publisher(mp_publisher);
    }

    qDebug()  << "Removing DSRPublisher";
    */
}

std::tuple<bool, eprosima::fastdds::dds::Publisher*, eprosima::fastdds::dds::DataWriter*>
        DSRPublisher::init(eprosima::fastdds::dds::DomainParticipant *mp_participant_, eprosima::fastdds::dds::Topic *topic, bool isStreamData )
{
    mp_participant = mp_participant_;


    eprosima::fastdds::dds::DataWriterQos dataWriterQos;
    dataWriterQos.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    dataWriterQos.history().kind = eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
    dataWriterQos.resource_limits().max_samples = 300;
    dataWriterQos.resource_limits().allocated_samples = 300;
    dataWriterQos.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    dataWriterQos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;

    dataWriterQos.endpoint().history_memory_policy = DYNAMIC_REUSABLE_MEMORY_MODE;

    bool local = std::find_if(mp_participant_->get_qos().transport().user_transports.begin(),
                              mp_participant_->get_qos().transport().user_transports.end(),
                              [&](auto &transport) {
                                    return transport != nullptr && dynamic_cast<eprosima::fastdds::rtps::SharedMemTransportDescriptor*>(transport.get()) != nullptr;
                               }) != mp_participant_->get_qos().transport().user_transports.end();


    if (not local) {
        eprosima::fastrtps::rtps::Locator_t locator;
        locator.port = 7900;
        locator.kind = LOCATOR_KIND_UDPv4;
        eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, "239.255.1.33");
        dataWriterQos.endpoint().multicast_locator_list.push_back(locator);

    }



    ThroughputControllerDescriptor PublisherThroughputController{30000000, 1000};
    dataWriterQos.throughput_controller() = PublisherThroughputController;

    if (isStreamData) {
        dataWriterQos.reliability().kind = eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
        dataWriterQos.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
        dataWriterQos.history().depth = 50;
        //dataWriterQos.resource_limits().allocated_samples = 300;
        dataWriterQos.reliable_writer_qos().disable_positive_acks.enabled = true;
    }

    // Check ACK for sended messages.
    dataWriterQos.reliable_writer_qos().times.heartbeatPeriod.seconds = 0;
    dataWriterQos.reliable_writer_qos().times.heartbeatPeriod.nanosec = 50000000; //50 ms. This value should be more or less close to the sending frequency.

    //Check latency
    dataWriterQos.latency_budget().duration = {0,50000000}; //50ms;

    //Invalidate data after 1 second. If we dont receive it after this time we probably won't get it.
    dataWriterQos.lifespan().duration = 1;


    int retry = 0;
    while (retry < 5) {

        mp_publisher = mp_participant->create_publisher(eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);
        mp_writer = mp_publisher->create_datawriter(topic, dataWriterQos, &m_listener);

        if(mp_publisher != nullptr && mp_writer != nullptr) {
            qDebug() << "Publisher created, waiting for Subscribers." ;
            return { true, mp_publisher, mp_writer };
        }
        retry++;
        qDebug() << "Error creating publisher, retrying. [" << retry <<"/5]"  ;

    }

    qFatal("%s", std::string_view("Could not create publisher " + std::string(topic->get_name()) + " after 5 attempts").data());

}

eprosima::fastrtps::rtps::GUID_t DSRPublisher::getParticipantID() const
{
    return mp_participant->guid();
}


bool DSRPublisher::write(IDL::MvregNode *object)
{
    int retry = 0;
    while (retry < 5) {
        if (mp_writer->write(object)) return true;
        retry++;
    }
    qInfo() << "Error writing NODE " << object->id() << " after 5 attempts";
    return false;
}


bool DSRPublisher::write(IDL::MvregEdge *object)
{
    int retry = 0;
    while (retry < 5) {
        if (mp_writer->write(object)) return true;
        retry++;
    }
    qInfo() << "Error writing EDGE " << object->from() << " " << object->to() << " " << object->type().data() << " after 5 attempts";
    return false;
}


bool DSRPublisher::write(IDL::OrMap *object)
{
    int retry = 0;
    while (retry < 5) {
        if (mp_writer->write(object)) return true;
        retry++;
    }
    qInfo() << "Error writing GRAPH " << object->m().size() << " after 5 attempts";
    return false;
}

bool DSRPublisher::write(IDL::GraphRequest *object)
{
    int retry = 0;
    while (retry < 5) {
        if (mp_writer->write(object)) return true;
        retry++;
    }
    qInfo() << "Error writing GRAPH REQUEST after 5 attempts." ;
    return false;
}

bool DSRPublisher::write(std::vector<IDL::MvregEdgeAttr> *object)
{
    int retry = 0;
    while (retry < 5) {
        if (mp_writer->write(object)) return true;
        retry++;
    }
    qInfo() << "Error writing EDGE ATTRIBUTE VECTOR  after 5 attempts";
    return false;
}

bool DSRPublisher::write(std::vector<IDL::MvregNodeAttr> *object) {
    int retry = 0;
    while (retry < 5) {
        if (mp_writer->write(object)) return true;
        retry++;
    }
    qInfo() << "Error writing EDGE ATTRIBUTE VECTOR after 5 attempts";
    return false;
}
/*
void DSRPublisher::remove_publisher()
{
    if (mp_participant != nullptr) {
        if (mp_writer != nullptr)
        {
            mp_publisher->delete_datawriter(mp_writer);
            mp_writer = nullptr;
        }

        if (mp_publisher != nullptr)
        {
            mp_participant->delete_publisher(mp_publisher);
            mp_publisher = nullptr;
        }
    }
}
*/

void DSRPublisher::PubListener::on_publication_matched(eprosima::fastdds::dds::DataWriter* writer,
                                                       const eprosima::fastdds::dds::PublicationMatchedStatus& info)
{
    if (info.current_count == eprosima::fastrtps::rtps::MATCHED_MATCHING) {
        n_matched++;
        qInfo() << "Subscriber [" << writer->get_topic()->get_name().data() <<"] matched " << info.last_subscription_handle.value;// << " self: " << info.remoteEndpointGuid.is_on_same_process_as(pub->getGuid());
    } else {
        n_matched--;
        qInfo() << "Subscriber [" << writer->get_topic()->get_name().data() <<"] unmatched" << info.last_subscription_handle.value;// << " self: " <<info.remoteEndpointGuid.is_on_same_process_as(pub->getGuid());
    }
}
