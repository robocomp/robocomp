#include <Ice/Ice.h>
#include <LaserI.h>
 
using namespace std;
using namespace RoboCompLaser;
 
int main(int argc, char* argv[]) 
{ 
    Ice::CommunicatorPtr ic;
    int status = 0;
    try
    {
        ic = Ice::initialize(argc, argv);
        auto adapter = ic->createObjectAdapterWithEndpoints("RoboCompLaserAdapter", "default -p 10000");
        Ice::ObjectPtr object = new LaserI(argc, argv);
        adapter->add(object, Ice::stringToIdentity("RoboCompLaser"));
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
