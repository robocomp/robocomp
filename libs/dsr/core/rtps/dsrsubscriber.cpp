#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastrtps/utils/IPFinder.h>

#include <dsr/core/rtps/dsrsubscriber.h>

#include <QDebug>

using namespace eprosima;
using namespace eprosima::fastdds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

DSRSubscriber::DSRSubscriber() : mp_participant(nullptr), mp_subscriber(nullptr), mp_reader(nullptr) {}

DSRSubscriber::~DSRSubscriber()
{
}

std::tuple<bool, eprosima::fastdds::dds::Subscriber*, eprosima::fastdds::dds::DataReader*>
        DSRSubscriber::init(eprosima::fastdds::dds::DomainParticipant *mp_participant_,
                         eprosima::fastdds::dds::Topic *topic,
                        const std::function<void(eprosima::fastdds::dds::DataReader*)>&  f_,
                        std::mutex& mtx,
                        bool isStreamData)
{
    mp_participant = mp_participant_;


    //m_listener.participant_ID = mp_participant->guid();
    m_listener.f = f_;


    eprosima::fastdds::dds::SubscriberQos Rparam;


    eprosima::fastdds::dds::DataReaderQos dataReaderQos;
    dataReaderQos.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    dataReaderQos.history().kind = eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
    dataReaderQos.resource_limits().max_samples = 400;
    dataReaderQos.resource_limits().allocated_samples = 400;
    dataReaderQos.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;

    dataReaderQos.endpoint().history_memory_policy = DYNAMIC_REUSABLE_MEMORY_MODE;

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
        dataReaderQos.endpoint().multicast_locator_list.push_back(locator);
    }

    //Check latency
    dataReaderQos.latency_budget().duration = {0,50000000}; //50ms;

    if (isStreamData) {
        dataReaderQos.reliability().kind = eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
        dataReaderQos.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
        dataReaderQos.history().depth = 50;
        dataReaderQos.reliable_reader_qos().disable_positive_ACKs.enabled = true;
    }



    int retry = 0;
    std::unique_lock<std::mutex> lck (mtx, std::defer_lock);
    while (retry < 5) {
        lck.lock();
        mp_subscriber = mp_participant->create_subscriber(Rparam);
        mp_reader = mp_subscriber->create_datareader(topic, dataReaderQos , &m_listener);
        lck.unlock();
        //mp_subscriber = Domain::createSubscriber(mp_participant, Rparam, static_cast<SubscriberListener*>(&m_listener));
        if (mp_subscriber != nullptr && mp_reader != nullptr) {
            qDebug() << "Subscriber created, waiting for Publishers." ;
            return { true, mp_subscriber, mp_reader };
        }
        retry++;
        qDebug() << "Error creating Subscriber, retrying. [" << retry <<"/5]"  ;
    }

    qFatal("%s", std::string_view("Could not create subscriber " + std::string(topic->get_name()) + " after 5 attempts").data());

}


eprosima::fastdds::dds::Subscriber * DSRSubscriber::getSubscriber(){
    return mp_subscriber;
}

eprosima::fastdds::dds::DataReader * DSRSubscriber::getDataReader() {
    return mp_reader;
}

///////////////////////////////////////////
/// Callbacks
///////////////////////////////////////////

void DSRSubscriber::SubListener::on_subscription_matched(eprosima::fastdds::dds::DataReader* reader,
                                                         const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
{
    if (info.current_count_change == eprosima::fastrtps::rtps::MATCHED_MATCHING)
    {
        //n_matched++;
        qInfo() << "Publisher[" << reader->get_topicdescription()->get_name().data() <<"] matched " << info.last_publication_handle.value;// << " self: " << info..is_on_same_process_as(sub->getGuid());
    } else {
        //n_matched--;
        qInfo() << "Publisher[" << reader->get_topicdescription()->get_name().data() <<"] unmatched "  << info.last_publication_handle.value;//<< " self: " << info.remoteEndpointGuid.is_on_same_process_as(sub->getGuid());
    }
}

void DSRSubscriber::SubListener::on_data_available(eprosima::fastdds::dds::DataReader* sub)
{
    f(sub);
}

