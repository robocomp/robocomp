// ------------------------------------------------------------------------------------------------
// Laser.ice
// ------------------------------------------------------------------------------------------------
#include "specificworker.h"
// TLaserData SpecificWorker::laser_getLaserData ( const QString& server )
// {
// 	QMutexLocker l ( mutex );
// 	
// 	IMVLaser &las = imv->lasers[server];
// 	QString laserConfig = las.laserNode->ifconfig;
// 	uint32_t basePort  = laserConfig.toUInt();
// 	RoboCompGenericBase::TBaseState bState;
// // 	for ( uint32_t s=0; s<handlerDifferentialRobots->servers.size(); ++s ) {
// // 		if ( handlerDifferentialRobots->servers[s].port == basePort ) {
// // 			( ( handlerDifferentialRobots->servers[s].interface ) )->getBaseState ( bState );
// // 			break;
// // 		}
// // 	}
// 	
// 	std::map<uint32_t, DifferentialRobotServer>::iterator it = handlerDifferentialRobots->servers.find( basePort );
// 	if( it != handlerDifferentialRobots->servers.end() ) {
// 		it->second.interface->getBaseState ( bState );
// 	}
// 	
// 	if ( laserDataArray.contains ( server ) == true ) {
// 		return laserDataArray[server];
// 	} else {
// 		RoboCompLaser::TLaserData l;
// 		qDebug() << "Error returning TLaserData"; //SHOULD RETURN A LASER EXCEPTION
// 		return l;
// 	}
// }


TLaserData SpecificWorker::laser_getLaserAndBStateData ( const QString& server, RoboCompGenericBase::TBaseState& state )
{
	//QMutexLocker l ( mutex );
	guard gl(innerModel->mutex);

		IMVLaser &las = imv->lasers[server];
		QString laserConfig = las.laserNode->ifconfig;
		uint32_t basePort  = laserConfig.toUInt();
		
	// 	for ( uint32_t s=0; s<handlerDifferentialRobots->servers.size(); ++s ) {
	// 		if ( handlerDifferentialRobots->servers[s].port == basePort ) {
	// 			( ( handlerDifferentialRobots->servers[s].interface ) )->getBaseState ( state );
	// 			break;
	// 		}
	// 	}
		
		std::map<uint32_t, DifferentialRobotServer>::iterator it = dfr_servers.find( basePort );
		if( it != dfr_servers.end() ) 
		{
			it->second.interface->getBaseState ( state );
		}
		
		if ( laserDataArray.contains ( server ) == true ) 
		{
			return laserDataArray[server];
		} 
		else 
		{
			RoboCompLaser::TLaserData l;
			qDebug() << "Error returning TLaserData"; //SHOULD RETURN A LASER EXCEPTION
			return l;
		}
}


LaserConfData SpecificWorker::laser_getLaserConfData ( const QString& server )
{
	LaserConfData lcd;
	return lcd;
}
