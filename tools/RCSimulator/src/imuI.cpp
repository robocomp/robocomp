#include "imuI.h"
#include "specificworker.h"


IMUI::IMUI ( SpecificWorker *_worker, QObject *parent ) : QObject ( parent )
{
	worker = _worker;
}


void IMUI::add ( QString id )
{
	imuIDs << id;
}


IMUI::~IMUI()
{
}


void IMUI::updateIMUData ( QString id )
{
	worker->imu_updateIMUData( imuIDs[0], id );
}


DataImu IMUI::getDataImu ( const Ice::Current& )
{
	return worker->imu_getDataImu( imuIDs[0] );
}


Acceleration IMUI::getAcceleration ( const Ice::Current& )
{
	return getDataImu().acc;
}


Gyroscope IMUI::getAngularVel ( const Ice::Current& )
{
	return getDataImu().gyro;
}


Magnetic IMUI::getMagneticFields ( const Ice::Current& )
{
	return getDataImu().mag;
}


Orientation IMUI::getOrientation ( const Ice::Current& )
{
	return getDataImu().rot;
}


void IMUI::resetImu ( const Ice::Current& )
{
	worker->imu_resetImu( imuIDs[0] );
}
