#include <iostream>
#include <termios.h>

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

#include "diffdrive_state.pb.h"
#include "diffdrive_cmd.pb.h"

char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		std::cerr << "tcsetattr()" << std::endl;
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		std::cerr<< "tcsetattr ICANON" << std::endl;

	if(rv == -1)
		std::cerr<< "select" << std::endl;
	else if(rv == 0)
		std::cout << "no_key_pressed" << std::endl;
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		std::cerr << "tcsetattr ~ICANON" << std::endl;
	return (buff);
}

int main(int argc, char** argv) {
    // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(argc, argv);
#else
    gazebo::client::setup(argc, argv);
#endif

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the  velodyne topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<diffdrive_cmd::msgs::DiffDriveCmd>("/my_robot");

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Create a a vector3 message
    diffdrive_cmd::msgs::DiffDriveCmd msg;

    while (true) {

        int keypressed = 0;
		keypressed = getch();

        if (keypressed == 'w') {
            msg.set_linear_vel(10);
            msg.set_angular_vel(0);
        }
        if (keypressed == 'a') {
            msg.set_linear_vel(10);
            msg.set_angular_vel(5);
        }
        if (keypressed == 'd') {
            msg.set_linear_vel(10);
            msg.set_angular_vel(-5);
        }
        if (keypressed == 's') {
            msg.set_linear_vel(-10);
            msg.set_angular_vel(0);
        }
        
        // Send the message
        pub->Publish(msg);
        gazebo::common::Time::MSleep(10);
    }

    // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
#else
    gazebo::client::shutdown();
#endif

    return 0;
}