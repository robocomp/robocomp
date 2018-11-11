#include "imuI.h"
#include "specificworker.h"


IMUI::IMUI (std::shared_ptr<SpecificWorker> _worker, QObject *parent ) : QObject ( parent )
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
	guard gl(worker->innerModel->mutex);

	QMat R = worker->innerModel->getRotationMatrixTo(id, "root");
	QVec acc   = R * QVec::vec3(0.0, -9.80665,  0.0);
	QVec north = R * QVec::vec3(0.0,      0.0, 10.0);

	data_imu.acc.XAcc = acc(0);
	data_imu.acc.YAcc = acc(1);
	data_imu.acc.ZAcc = acc(2);

	data_imu.rot.Pitch = atan2(acc(2),    -acc(1));
	data_imu.rot.Roll  = atan2(acc(0),    -acc(1));
	data_imu.rot.Yaw   = atan2(north(0), north(2));
	data_imu.temperature = 25.;
}

DataImu IMUI::getDataImu ( const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	this->updateIMUData(imuIDs[0]);
	return data_imu;	
}

Acceleration IMUI::getAcceleration ( const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	this->updateIMUData(imuIDs[0]);
	return this->getDataImu().acc;
}

Gyroscope IMUI::getAngularVel ( const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	this->updateIMUData(imuIDs[0]);
	return this->getDataImu().gyro;
}

Magnetic IMUI::getMagneticFields ( const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	this->updateIMUData(imuIDs[0]);
	return this->getDataImu().mag;
}

Orientation IMUI::getOrientation ( const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	this->updateIMUData(imuIDs[0]);
	return this->getDataImu().rot;
}

void IMUI::resetImu ( const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	this->updateIMUData(imuIDs[0]);
}
