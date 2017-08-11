// ------------------------------------------------------------------------------------------------
// IMU.ice
// ------------------------------------------------------------------------------------------------

void SpecificWorker::imu_updateIMUData(const QString& server, QString id)
{
	QMutexLocker locker(mutex);

	QMat R = innerModel->getRotationMatrixTo(id, "root");
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


DataImu SpecificWorker::imu_getDataImu(const QString& server)
{
	return data_imu;
}


void SpecificWorker::imu_resetImu(const QString& server)
{
}
