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

int
main(int argc, char* argv[])
{
	int status = 0;		// 0 if Communication link is proper
	Ice::CommunicatorPtr client;   
	VideoCapture cap(0); 
	if(!cap.isOpened())  
		return -1;
	try 
	{
		namedWindow("frame",1);	
		namedWindow("My Window", 1);
		int value_0_0 = 0;
		createTrackbar("value_0_0", "My Window", &value_0_0, 2);
		Mat frame;
		Mat gray;
		client = Ice::initialize(argc, argv);
		Ice::ObjectPrx base = client->stringToProxy("SimplePrinter:default -p 10000");
		PrinterPrx printer = PrinterPrx::checkedCast(base);
		if (!printer)
			throw "Invalid proxy";
		printer->printString("Starting video!");
		while (true)
		{
			int value000 = value_0_0;
			cap >> frame; // get a new frame from camera	
			Mat result;	
			if(value000 == 0)
			{
				result = frame.clone();
				printer->printString("Showing normal video !");
			}
			else if(value000 == 1)
			{
				cvtColor(frame,gray,CV_BGR2GRAY);
				result = gray.clone();
				printer->printString("Actual video converted to grayscale video !");			
			}
			else
			{
				cvtColor(frame,gray,CV_BGR2GRAY);
				result = gray.clone();
				Canny( gray, result, 20, 60, 3 );
				printer->printString("Showing edges of the actual video!");
			}
			imshow("frame",result);
			if(waitKey(30) >= 0) break; 
		}		
	} 
	catch (const Ice::Exception& ex) 
	{
		cerr << ex << endl;
		status = 1;
	}
	catch (const char* msg)
	{
		cerr << msg << endl;
		status = 1;
	}
	if (client)
	{
		client->destroy();
	}
			
	return status;
}
