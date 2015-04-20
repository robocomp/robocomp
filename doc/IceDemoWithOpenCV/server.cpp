/****
	Author - Abhishek Kumar Annamraju
****/

#include <Ice/Ice.h>
#include "Printer.h"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>


using namespace std;
using namespace Demo;
using namespace cv;


class PrinterI : public Printer 
{
	public:
		virtual void printString(const string& s, const Ice::Current&);
};

void PrinterI::printString(const string& s, const Ice::Current&)
{
	cout << s << endl;
}

int main(int argc, char* argv[])
{
	int status = 0;
	Ice::CommunicatorPtr server;
    
	try
	{
		server = Ice::initialize(argc, argv);
		Ice::ObjectAdapterPtr adapter = server->createObjectAdapterWithEndpoints("SimplePrinterAdapter", "default -p 10000");
		Ice::ObjectPtr object = new PrinterI;
		adapter->add(object, server->stringToIdentity("SimplePrinter"));
		adapter->activate();
		server->waitForShutdown();
	}
	catch (const Ice::Exception& e) 
	{
		cerr << e << endl;
		status = 1;
	} 
	catch (const char* msg)
	{
		cerr << msg << endl;
		status = 1;
	}
	if (server)
	{
		try 
		{
			server->destroy();
		}
		catch (const Ice::Exception& e) 
		{
			cerr << e << endl;
			status = 1;
		}
	}
	return status;
}
