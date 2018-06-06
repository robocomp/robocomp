#include "IMUI.h"
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

using namespace RoboCompIMU;
using namespace std;
using namespace gazebo; 

IMUI::IMUI(int argc, char **argv) {
    gazebo::client::setup(argc, argv);
    this->device_name_ = "gazebo_robocomp_IMU";
    this->topic_name_ = "/gazebo/IMU/data";
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init();
    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(topic_name_, &IMUI::callback, this);

    this->imu_data_.acc.XAcc = 0;
    this->imu_data_.acc.YAcc = 0;
    this->imu_data_.acc.ZAcc = 0;

    this->imu_data_.gyro.XGyr = 0;
    this->imu_data_.gyro.YGyr = 0;
    this->imu_data_.gyro.ZGyr = 0;

    this->imu_data_.mag.XMag = 0;
    this->imu_data_.mag.YMag = 0;
    this->imu_data_.mag.ZMag = 0;

    this->imu_data_.rot.Roll = 0;
    this->imu_data_.rot.Pitch = 0;
    this->imu_data_.rot.Yaw = 0;

    this->imu_data_.temperature = 0;
} 

IMUI::~IMUI() {
    gazebo::client::shutdown();
}

DataImu IMUI::getDataImu(const Ice::Current&) {
    return this->imu_data_;
}

Acceleration IMUI::getAcceleration(const Ice::Current&) {
    RoboCompIMU::Acceleration accln;
    accln.XAcc = this->imu_data_.acc.XAcc;
    accln.YAcc = this->imu_data_.acc.YAcc;
    accln.ZAcc = this->imu_data_.acc.ZAcc;
    return accln;
}

Gyroscope IMUI::getAngularVel(const Ice::Current&) {
    RoboCompIMU::Gyroscope gyr;
    gyr.XGyr = this->imu_data_.gyro.XGyr;
    gyr.YGyr = this->imu_data_.gyro.YGyr;
    gyr.ZGyr = this->imu_data_.gyro.ZGyr;
    return gyr;
}

Magnetic IMUI::getMagneticFields(const Ice::Current&) {
    RoboCompIMU::Magnetic mag;
    mag.XMag = this->imu_data_.mag.XMag;
    mag.YMag = this->imu_data_.mag.YMag;
    mag.ZMag = this->imu_data_.mag.ZMag;
    return mag;
}

Orientation IMUI::getOrientation(const Ice::Current&) {
    RoboCompIMU::Orientation orient;
    orient.Roll = this->imu_data_.rot.Roll;
    orient.Pitch = this->imu_data_.rot.Pitch;
    orient.Yaw = this->imu_data_.rot.Yaw;
    return orient;
}

void IMUI::resetImu(const Ice::Current&) {
    this->imu_data_.acc.XAcc = 0;
    this->imu_data_.acc.YAcc = 0;
    this->imu_data_.acc.ZAcc = 0;

    this->imu_data_.gyro.XGyr = 0;
    this->imu_data_.gyro.YGyr = 0;
    this->imu_data_.gyro.ZGyr = 0;

    this->imu_data_.mag.XMag = 0;
    this->imu_data_.mag.YMag = 0;
    this->imu_data_.mag.ZMag = 0;

    this->imu_data_.rot.Roll = 0;
    this->imu_data_.rot.Pitch = 0;
    this->imu_data_.rot.Yaw = 0;

    this->imu_data_.temperature = 0;
}

void IMUI::callback(ConstIMUPtr &_msg) {
    this->imu_data_.acc.XAcc = _msg->linear_acceleration().x();
    this->imu_data_.acc.YAcc = _msg->linear_acceleration().y();
    this->imu_data_.acc.ZAcc = _msg->linear_acceleration().z();

    this->imu_data_.gyro.XGyr = _msg->angular_velocity().x();
    this->imu_data_.gyro.YGyr = _msg->angular_velocity().y();
    this->imu_data_.gyro.ZGyr = _msg->angular_velocity().z();

    this->imu_data_.mag.XMag = 0;
    this->imu_data_.mag.YMag = 0;
    this->imu_data_.mag.ZMag = 0;

    this->imu_data_.rot.Roll = _msg->orientation().x();
    this->imu_data_.rot.Pitch = _msg->orientation().y();
    this->imu_data_.rot.Yaw = _msg->orientation().z();

    this->imu_data_.temperature = 0;
} 

