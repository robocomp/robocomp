#ifndef _SUBSCRIBER_H_
#define _SUBSCRIBER_H_

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include <functional>


class DSRSubscriber
{
public:
	DSRSubscriber();
	virtual ~DSRSubscriber();
    [[nodiscard]] std::tuple<bool, eprosima::fastdds::dds::Subscriber*, eprosima::fastdds::dds::DataReader*>
	          init(eprosima::fastdds::dds::DomainParticipant *mp_participant_,
                   eprosima::fastdds::dds::Topic *topic,
				   const std::function<void(eprosima::fastdds::dds::DataReader*)>&  f_,
				   std::mutex& mtx,
				   bool isStreamData = false);
	//void run();
    eprosima::fastdds::dds::Subscriber *getSubscriber();
    eprosima::fastdds::dds::DataReader *getDataReader();
    //void remove_subscriber();

private:
    eprosima::fastdds::dds::DomainParticipant *mp_participant;
    eprosima::fastdds::dds::Subscriber *mp_subscriber;
    eprosima::fastdds::dds::DataReader *mp_reader{};

	class SubListener : public eprosima::fastdds::dds::DataReaderListener
	{
	public:
		SubListener() = default;
		~SubListener() override= default;
		void on_subscription_matched(eprosima::fastdds::dds::DataReader* reader,
                                     const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;
        void on_data_available(
                eprosima::fastdds::dds::DataReader* reader) override;

		std::function<void(eprosima::fastdds::dds::DataReader* sub)>  f;

	} m_listener;

};

#endif // _SUBSCRIBER_H_