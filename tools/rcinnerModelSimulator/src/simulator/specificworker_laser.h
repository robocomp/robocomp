// ------------------------------------------------------------------------------------------------
// Laser.ice
// ------------------------------------------------------------------------------------------------

// TLaserData SpecificWorker::laser_getLaserData ( const QString& server )
// {
// 	QMutexLocker l ( mutex );
// 	
// 	IMVLaser &las = d->imv->lasers[server];
// 	QString laserConfig = las.laserNode->ifconfig;
// 	uint32_t basePort  = laserConfig.toUInt();
// 	RoboCompGenericBase::TBaseState bState;
// // 	for ( uint32_t s=0; s<d->handlerDifferentialRobots->servers.size(); ++s ) {
// // 		if ( d->handlerDifferentialRobots->servers[s].port == basePort ) {
// // 			( ( d->handlerDifferentialRobots->servers[s].interface ) )->getBaseState ( bState );
// // 			break;
// // 		}
// // 	}
// 	
// 	std::map<uint32_t, DifferentialRobotServer>::iterator it = d->handlerDifferentialRobots->servers.find( basePort );
// 	if( it != d->handlerDifferentialRobots->servers.end() ) {
// 		it->second.interface->getBaseState ( bState );
// 	}
// 	
// 	if ( d->laserDataArray.contains ( server ) == true ) {
// 		return d->laserDataArray[server];
// 	} else {
// 		RoboCompLaser::TLaserData l;
// 		qDebug() << "Error returning TLaserData"; //SHOULD RETURN A LASER EXCEPTION
// 		return l;
// 	}
// }


TLaserData SpecificWorker::laser_getLaserAndBStateData ( const QString& server, RoboCompGenericBase::TBaseState& state )
{
	QMutexLocker l ( mutex );
	
	IMVLaser &las = d->imv->lasers[server];
	QString laserConfig = las.laserNode->ifconfig;
	uint32_t basePort  = laserConfig.toUInt();
	
// 	for ( uint32_t s=0; s<d->handlerDifferentialRobots->servers.size(); ++s ) {
// 		if ( d->handlerDifferentialRobots->servers[s].port == basePort ) {
// 			( ( d->handlerDifferentialRobots->servers[s].interface ) )->getBaseState ( state );
// 			break;
// 		}
// 	}
	
	std::map<uint32_t, DifferentialRobotServer>::iterator it = d->dfr_servers.find( basePort );
	if( it != d->dfr_servers.end() ) {
		it->second.interface->getBaseState ( state );
	}
	
	if ( d->laserDataArray.contains ( server ) == true ) {
		return d->laserDataArray[server];
	} else {
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
