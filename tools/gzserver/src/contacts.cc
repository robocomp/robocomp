#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

void OnMsg(ConstContactsPtr &_contacts){

    for (unsigned int i = 0; i < contacts.contact_size(); ++i)
    {
    	std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";

    	for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    	{
            std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
            std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
            std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
    	}
    }

}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
    // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(_argc, _argv);
#else
    gazebo::client::setup(_argc, _argv);
#endif

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::transport::SubscriberPtr sub = node->Subscibe("/robocomp/contacts", &OnMsg);

    while (true) {
        gazebo::common::Time::MSleep(10);
    }

    // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
#else
    gazebo::client::shutdown();
#endif
}
