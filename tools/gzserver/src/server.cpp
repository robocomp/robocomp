#include <Ice/Ice.h>
#include <LaserI.h>
#include <MotorI.h>
#include <RGBDI.h>
#include <JointMotorI.h>
#include <IMUI.h>
#include <DifferentialRobotI.h>
#include <CameraI.h>
#include <bumperI.h>
 
using namespace std;
using namespace RoboCompLaser;
 
int main(int argc, char* argv[]) 
{ 
    Ice::CommunicatorPtr ic;
    int status = 0;
    try
    {
        ic = Ice::initialize(argc, argv);
        auto adapter = ic->createObjectAdapterWithEndpoints("GazeboServerAdapter", "default -p 10000");
        
        Ice::ObjectPtr laser = new LaserI(argc, argv);
        Ice::ObjectPtr RGBD = new RGBDI(argc, argv);
        Ice::ObjectPtr camera = new CameraI(argc, argv);
        Ice::ObjectPtr bumper = new bumperI(argc, argv);
        Ice::ObjectPtr differentialrobot = new DifferentialRobotI(argc, argv);
        Ice::ObjectPtr jointmotor = new JointMotorI(argc, argv);
        Ice::ObjectPtr imu = new IMUI(argc, argv);
        Ice::ObjectPtr motor = new MotorI(argc, argv);

        adapter->add(laser, Ice::stringToIdentity("RoboCompLaser"));
        adapter->add(RGBD, Ice::stringToIdentity("RoboCompRGBD"));
        adapter->add(camera, Ice::stringToIdentity("RoboCompCamera"));
        adapter->add(bumper, Ice::stringToIdentity("RoboCompBumper"));
        adapter->add(differentialrobot, Ice::stringToIdentity("RoboCompDifferentialRobot"));
        adapter->add(imu, Ice::stringToIdentity("RoboCompIMU"));
        adapter->add(bumper, Ice::stringToIdentity("RoboCompBumper"));
        adapter->add(motor, Ice::stringToIdentity("RoboCompMotor"));

        adapter->activate();
        ic->waitForShutdown();
    }
    catch(const Ice::Exception& e)
    {
        cerr << e << endl;
        status = 1;
    }

    catch (const std::exception& e) 
    {
        cerr << e.what() << endl;
        status = 1;
    }

    if(ic) 
    {
        try 
        {
            ic->destroy();
        } 
        catch(const Ice::Exception& e) 
        {
            cerr << e << endl;
            status = 1;
        }
    }
    return 0;
}
