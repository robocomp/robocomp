// ------------------------------------------------------------------------------------------------
// RGBD.ice
// ------------------------------------------------------------------------------------------------

// 	throw std::string( __PRETTY_FUNCTION__ ) + std::string( " not implemented yet!" );

TRGBDParams SpecificWorker::rgbd_getRGBDParams ( const QString& server )
{
	//QMutexLocker locker ( mutex );
	guard gl(innerModel->mutex);
	
	IMVCamera &cam = imv->cameras[server];

	RoboCompRGBD::TRGBDParams rgbdParams;
	rgbdParams.driver = "RCIS";
	rgbdParams.device = server.toStdString();
	rgbdParams.timerPeriod = timer.interval();

	QStringList cameraConfig = cam.RGBDNode->ifconfig.split ( "," );
	if ( cameraConfig.size() > 1 ) {
		uint32_t basePort  = QString ( cameraConfig[1] ).toUInt();
		if( dfr_servers.count( basePort ) > 0 )
			rgbdParams.talkToBase = true;
		
		uint32_t jointPort = QString ( cameraConfig[0] ).toUInt();
		if( jm_servers.count( jointPort ) > 0 )
			rgbdParams.talkToJointMotor = true;
	}

	RoboCompRGBD::CameraParameters camParams;
	camParams.focal = cam.RGBDNode->focal;
	camParams.width = cam.RGBDNode->width;
	camParams.height = cam.RGBDNode->height;
	camParams.size = camParams.width*camParams.height;
	camParams.FPS = rgbdParams.timerPeriod;
	rgbdParams.color = camParams;
	rgbdParams.depth = camParams;

	return rgbdParams;
}


void SpecificWorker::rgbd_setRegistration ( const QString& server, Registration value )
{
	//QMutexLocker locker ( mutex );
	guard gl(innerModel->mutex);
}


Registration SpecificWorker::rgbd_getRegistration ( const QString& server )
{
	//QMutexLocker locker ( mutex );
	guard gl(innerModel->mutex);

	return RoboCompRGBD::DepthInColor;
}


void SpecificWorker::rgbd_getData ( const QString& server, RoboCompRGBD::imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState )
{
	//QMutexLocker locker ( mutex );
	guard gl(innerModel->mutex);
	
	ColorSeq color;
	DepthSeq depth;
	PointSeq points;
	this->rgbd_getImage ( server, color, depth, points, hState, bState );
	
	rgbMatrix.resize ( 640*480*3 );
	distanceMatrix.resize ( 640*480 );
	for ( int i=0; i<640*480; i++ ) {
		rgbMatrix[3*i+0] = color[i].red;
		rgbMatrix[3*i+1] = color[i].green;
		rgbMatrix[3*i+2] = color[i].blue;
		distanceMatrix[i] = depth[i];
	}
}


void SpecificWorker::rgbd_getImage ( const QString& server, ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState )
{
	//QMutexLocker locker ( mutex );
	guard gl(innerModel->mutex);

	IMVCamera &cam = imv->cameras[server];

	
	QStringList cameraConfig = cam.RGBDNode->ifconfig.split ( "," );
	if (cameraConfig.size() > 1)
	{
		uint32_t basePort  = QString ( cam.RGBDNode->ifconfig.split ( "," ) [1] ).toUInt();
		std::map<uint32_t, OmniRobotServer>::iterator base;
		base = omn_servers.find( basePort );
                bool bstateUpd = false;
		if (base != omn_servers.end())
		{
			base->second.interface->getBaseState( bState );
			bstateUpd = true;
		}
		std::map<uint32_t, DifferentialRobotServer>::iterator baseD;
		baseD = dfr_servers.find( basePort );
		if (baseD != dfr_servers.end())
		{
			baseD->second.interface->getBaseState( bState );
                        bstateUpd = true;
		}
		if (not bstateUpd)
                {
                    std::cout<<"Error: no base state updated, basePort "<<basePort<<std::endl;
                }
		uint32_t jointPort = QString ( cam.RGBDNode->ifconfig.split ( "," ) [0] ).toUInt();
		std::map<uint32_t, JointMotorServer>::iterator joint;
		joint = jm_servers.find( jointPort );
		if ( joint != jm_servers.end() )
		{
			RoboCompJointMotor::MotorStateMap newMap;
			joint->second.interface->getAllMotorState ( newMap );
			static RoboCompJointMotor::MotorStateMap backMap = newMap;
			hState = newMap;
			backMap = newMap;
		}
	}

	const int width = cam.RGBDNode->width;
	const int height = cam.RGBDNode->height;
	const float noise = ( float ) cam.RGBDNode->noise;
	const float focal = ( float ) cam.RGBDNode->focal;
	double fovy, aspectRatio, Zn, Zf;
	cam.viewerCamera->getCamera()->getProjectionMatrixAsPerspective ( fovy, aspectRatio, Zn, Zf );
//	printf("fov: %g, aspect: %g\n", fovy, aspectRatio);

	static QVec rndm = QVec::gaussianSamples(1000001, 1, noise);
	static bool rndmInit = false;
	if (not rndmInit)
	{
		rndmInit = true;
		for (int i=0; i<1000001; i++)
		{
			if (rndm(i) > 1.+noise*5.)
				 rndm(i) = 1.+noise*5.;
			if (rndm(i) <-1.-noise*5.)
				 rndm(i) =-1.-noise*5;
		}
	}

	if ( color.size() != ( uint ) width*height ) {
		color.resize ( width*height );
	}
	if ( depth.size() != ( uint ) width*height ) {
		depth.resize ( width*height );
	}
	if ( points.size() != ( uint ) width*height ) {
		points.resize ( width*height );
	}

	const unsigned char *rgb = cam.rgb->data();
	const float *d = ( float * ) cam.d->data();
	uint32_t td=0;
	for ( int i=0; i<height; ++i ) {
		for ( int j=0; j<width; ++j ) {
			const int index  = j + ( i ) *width;
			const int indexI = j + ( height-1-i ) *width;
			color[index].red         = rgb[3*indexI+0];
			color[index].green       = rgb[3*indexI+1];
			color[index].blue        = rgb[3*indexI+2];
			if ( d[indexI] <= 1. ) {
				depth[index] = ( Zn*Zf / ( Zf - d[indexI]* ( Zf-Zn ) ) ) *rndm((td++)%1000001) ;
			} else {
				depth[i] = NAN;
			}
			points[index].x = ( depth[index] * ( ( float ) j- ( width/2. ) ) / focal );
			points[index].y = depth[index] * ( (height/2.) - float(i) ) / focal;
			points[index].z = ( depth[index] );
			points[index].w = 1.;
		}
	}
}
