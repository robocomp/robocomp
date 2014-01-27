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
	updateIMUData(imuIDs[0]);
	return worker->imu_getDataImu( imuIDs[0] );
}


Acceleration IMUI::getAcceleration ( const Ice::Current& )
{
	updateIMUData(imuIDs[0]);
	return getDataImu().acc;
}


Gyroscope IMUI::getAngularVel ( const Ice::Current& )
{
	updateIMUData(imuIDs[0]);
	return getDataImu().gyro;
}


Magnetic IMUI::getMagneticFields ( const Ice::Current& )
{
	updateIMUData(imuIDs[0]);
	return getDataImu().mag;
}


Orientation IMUI::getOrientation ( const Ice::Current& )
{
	updateIMUData(imuIDs[0]);
	return getDataImu().rot;
}


void IMUI::resetImu ( const Ice::Current& )
{
	updateIMUData(imuIDs[0]);
	worker->imu_resetImu( imuIDs[0] );
}
