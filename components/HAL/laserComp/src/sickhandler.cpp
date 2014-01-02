#if COMPILE_SICK==1

#include "sickhandler.h"

#define RAD2DEG(x) (x*180/M_PI)  
#define DEG2RAD(x) (x*M_PI/180)  

SickLaserHandler::SickLaserHandler(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *_parent) : GenericLaserHandler (base_prx,_parent)
{
	laserConf = config;
	/* Instantiate the object */
	   base= base_prx;
	sicklms = new SickLMS("/dev/ttyUSB1"/*config.device*/);
	// Initialize the Sick LMS 2xx and then sets 
	// communication at the given baud rate. 
	try 
	{
		sicklms->Initialize(SickLMS::SICK_BAUD_500K);

		// Get the scan resolution currently being used by the device 
		scanres = DEG2RAD(sicklms->GetSickScanResolution());
		//Get the current measurement units of the device 
		units = sicklms->GetSickMeasuringUnits();

		// Div_factor will convert the measurement to meters
		if(units == SickLMS::SICK_MEASURING_UNITS_MM)
			div_factor = 1000;
		else if(units == SickLMS::SICK_MEASURING_UNITS_CM)
			div_factor = 100;

		printf("Operating mode: %X\n", sicklms->GetSickOperatingMode());

	}
	catch(...) 
	{
		std::cerr << "No se pudo conectar con el láser. Asegúrese de que el láser "
					 "está conectado por el puerto adecuado (p.ej /dev/ttyUSB0, ...)" << std::endl;
		exit();
	}
	laserData.resize(400);
 
}
SickLaserHandler::~SickLaserHandler()
{
    //Uninitialize the LMS by putting it in a mode where it stops 
	//streaming data, and return it to the default baud rate
	try 
	{
		sicklms->Uninitialize();
	}
	catch(...) 
	{
		std::cerr << "No se pudo desconectar correctamente el láser." << std::endl;
	}
}


void SickLaserHandler::run()
{
	forever
	{
		if (readLaserData()==false)
		{
			std::cout << "Error reading laser " << std::endl;
		}
		usleep(10000);

	}
  
}
bool SickLaserHandler::readLaserData()
{
	// Read laser data (polar)
// 	laserData.clear();
	try 
	{
		sicklms->GetSickScan(polarvalues, num_values);
		//laserPolarData.resize(num_values);
			
		if(num_values > 361) num_values = 361;
		for(unsigned int i = 0; i < num_values; i++)
		{
			laserPolarData[i] = (double)polarvalues[i]/div_factor;
			laserData[i].angle = (180- i*0.5)*M_PI/180.f;
			laserData[i].dist = (double)polarvalues[i]/div_factor*1000;
		}
	}
    // Catch possible exceptions
  	catch(...) 
	{
		std::cerr << "Error while receiving laser data\n" << std::endl;
		return false;
  	}  
  	return true;
}


RoboCompLaser::TLaserData SickLaserHandler::getNewData()
{
	return laserData;
}
RoboCompLaser::LaserConfData SickLaserHandler::getLaserConf()
{
	return laserConf;  
}



#endif
