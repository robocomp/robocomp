#include <Ice/Ice.h>
#include "Laser.h"
#include <stdexcept>
 
using namespace std;
using namespace RoboCompLaser;
 
void printConfigData(LaserConfData &laser_data)
{
    std::cerr << "Max Measures: " << laser_data.maxMeasures << std::endl;
    std::cerr << "Max Degrees: " << laser_data.maxDegrees << std::endl;
    std::cerr << "Max Range: " << laser_data.maxRange << std::endl;
    std::cerr << "Min Range: " << laser_data.minRange << std::endl;
    std::cerr << "Initial Range: " << laser_data.iniRange << std::endl;
    std::cerr << "End Range: " << laser_data.endRange << std::endl;
    std::cerr << "Angle Resolution: " << laser_data.angleRes << std::endl;
    std::cerr << "Initial Angle: " << laser_data.angleIni << std::endl;
    std::cerr << "Device Diver: " << laser_data.driver << std::endl;
}
 
void printLaserScanData(TLaserData &laser_data)
{
    for(int i = 0; i < laser_data.size(); i++)
    {
        std::cerr << "Ray Index: " << i << " Ray Range: " << laser_data[i].dist << " Ray Angle: " << laser_data[i].angle << std::endl;
    }
}

int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        std::cerr << "Invalid Input!!!" << std::endl;
        return -1;
    }

    int toPublish = std::stoi(argv[1]);
    Ice::CommunicatorPtr ic;

    try
    {
        ic = Ice::initialize(argc, argv);
        Ice::ObjectPrx base = ic->stringToProxy("RoboCompLaser:default -p 10000");
        LaserPrx laser = LaserPrx::checkedCast(base);
        if(!laser)
        {
            throw std::runtime_error("Invalid proxy");
        }
 
        TLaserData scan_data = laser->getLaserData();
        LaserConfData config_data = laser->getLaserConfData();

        printConfigData(config_data);
        printLaserScanData(scan_data);

    }
    catch(const std::exception& e)
    {
        cerr << e.what() << endl;
        return 1;
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
            return 1;
        }
    }

    return 0;
}
