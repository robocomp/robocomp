#include <map>
#include <string>

#include "gazebo_robocomp_bumper.hh"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRoboCompBumper)

////////////////////////////////////////////////////////////////////////////////

// Constructor
GazeboRoboCompBumper::GazeboRoboCompBumper() {}
////////////////////////////////////////////////////////////////////////////////

// Destructor
GazeboRoboCompBumper::~GazeboRoboCompBumper() {}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRoboCompBumper::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
    // GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
    this->parent_sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(_parent);

    if (!this->parent_sensor_)
    {
        std::cerr << "bumper : Contact sensor parent is not of type ContactSensor"<< std::endl;
        return;
    }
 
    this->bumper_topic_name_ = "bumper_states";

    if (_sdf->HasElement("bumperTopicName"))
        this->bumper_topic_name_ = _sdf->GetElement("bumperTopicName")->Get<std::string>();
    else 
        std::cerr << "No element with <bumperTopicName> tag, setting it to default: " << bumper_topic_name_ << std::endl;

    this->gazebo_node_ = transport::NodePtr(new transport::Node());
    this->gazebo_node_->Init(parent_sensor_->WorldName());
    // this->pub_ = this->gazebo_node_->Advertise<gazebo::msgs::Contacts>(bumper_topic_name_);

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->update_connection_ = this->parent_sensor_->ConnectUpdated(boost::bind(&GazeboRoboCompBumper::OnContact, this));

    // Make sure the parent sensor is active.
    this->parent_sensor_->SetActive(true);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRoboCompBumper::OnContact()
{
    msgs::Contacts contacts; 
    contacts = this->parent_sensor_->Contacts();

    std::cerr << "publishing data on the topic: " << bumper_topic_name_ << std::endl;

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

    // this->pub_->Publish(contacts);
}

}
