#include <algorithm>
#include <string>
#include <assert.h>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/ImuSensor.hh>

#include "gazebo_robocomp_IMU.hh"

namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboRoboCompIMU)
  ////////////////////////////////////////////////////

  GazeboRoboCompIMU::GazeboRoboCompIMU(): SensorPlugin()
  {
    accelerometer_data = ignition::math::Vector3d(0, 0, 0);
    gyroscope_data = ignition::math::Vector3d(0, 0, 0);
    orientation = ignition::math::Quaterniond(1,0,0,0);
    sensor_=NULL;
  }

  GazeboRoboCompIMU::~GazeboRoboCompIMU() {}

  void GazeboRoboCompIMU::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    this->sdf_ = _sdf;
    this->world_name_ = _sensor->WorldName();

    this->sensor_ =  dynamic_cast<sensors::ImuSensor*>(_sensor.get());

    if (this->sensor_ == NULL)
    {
      gzthrow("ERROR: Sensor pointer is NULL");
      return;
    }

    this->sensor_->SetActive(true);

    // Topic to which the IMU data will be published
    if (this->sdf_->HasElement("topic"))
    {
      this->topic_name_ = sdf_->Get<std::string>("topic");
      std::cerr << "<topic> set to: "<< topic_name_ << std::endl;
    }
    else
    {
      this->topic_name_ = "/gazebo_robocomp_IMU/data";
      std::cerr << "missing <topic>, set to /namespace/default: " << topic_name_ << std::endl;
    }

    // Update Rate
    if (this->sdf_->HasElement("update_rate"))
    {
      update_rate =  sdf_->Get<double>("update_rate");
      std::cerr << "<update_rate> set to: " << update_rate << std::endl;
    }
    else
    {
      update_rate = 1.0;
      std::cerr << "missing <update_rate>, set to default: " << update_rate << std::endl;
    }

    // Create gazebo Node
    this->gazebo_node_ = transport::NodePtr(new  transport::Node());

    // Intialize the Node
    this->gazebo_node_->Init(this->world_name_);

    // Publisher to the topic
    imu_data_publisher_ = gazebo_node_->Advertise<gazebo::msgs::IMU>(topic_name_);

    connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRoboCompIMU::OnUpdate, this, _1));
    
    std::cerr << "IMU plugin loaded successfully!!!" << std::endl;
    std::cerr << "Orientation is representated in Quanternion" << std::endl;

  }

  void GazeboRoboCompIMU::OnUpdate(const common::UpdateInfo &)
  {

    orientation = sensor_->Orientation();
    accelerometer_data = sensor_->LinearAcceleration();
    gyroscope_data = sensor_->AngularVelocity();

    std::cerr << "++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "--------"<<"Getting Data---------" << std::endl;

    std::cerr << "Orientation_X: " << orientation.X() << std::endl;
    std::cerr << "Orientation_Y: " << orientation.Y() << std::endl;
    std::cerr << "Orientation_Z: " << orientation.Z() << std::endl;
    std::cerr << "Orientation_W: " << orientation.W() << std::endl;
    
    std::cerr << "Acceleration_X: " << accelerometer_data.X() << std::endl;
    std::cerr << "Acceleration_Y: " << accelerometer_data.Y() << std::endl;
    std::cerr << "Acceleration_Z: " << accelerometer_data.Z() << std::endl;

    std::cerr << "Gyroscope_X: " << gyroscope_data.X() << std::endl;
    std::cerr << "Gyroscope_Y: " << gyroscope_data.Y() << std::endl;
    std::cerr << "Gyroscope_Z: " << gyroscope_data.Z() << std::endl;

    gazebo::msgs::IMU msg;

    msg.mutable_stamp()->set_sec(this->sensor_->LastMeasurementTime().sec);
    msg.mutable_stamp()->set_nsec(this->sensor_->LastMeasurementTime().nsec);
    msg.set_entity_name("gazebo_robocomp_IMU");
    
    msg.mutable_orientation()->set_x(orientation.X());
    msg.mutable_orientation()->set_y(orientation.Y());
    msg.mutable_orientation()->set_z(orientation.Z());
    msg.mutable_orientation()->set_w(orientation.W());

    msg.mutable_angular_velocity()->set_x(gyroscope_data.X());
    msg.mutable_angular_velocity()->set_y(gyroscope_data.Y());
    msg.mutable_angular_velocity()->set_z(gyroscope_data.Z());

    msg.mutable_linear_acceleration()->set_x(accelerometer_data.X());
    msg.mutable_linear_acceleration()->set_y(accelerometer_data.Y());
    msg.mutable_linear_acceleration()->set_z(accelerometer_data.Z());

    imu_data_publisher_->Publish(msg);
  
  }

}
