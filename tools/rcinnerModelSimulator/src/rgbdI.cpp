/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "rgbdI.h"
#include "specificworker.h"

/**
* \brief Default constructor
*/
RGBDI::RGBDI ( std::shared_ptr<SpecificWorker> _worker, QObject *parent ) : QObject ( parent )
{
	worker = _worker;
}

/**
* \brief Default destructor
*/
RGBDI::~RGBDI()
{
}

void RGBDI::add ( QString _id )
{
	id = _id;
}


TRGBDParams RGBDI::getRGBDParams ( const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	
	IMVCamera &cam = worker->imv->cameras[id];

	RoboCompRGBD::TRGBDParams rgbdParams;
	rgbdParams.driver = "RCIS";
	rgbdParams.device = id.toStdString();
	rgbdParams.timerPeriod = worker->timer.interval();

	QStringList cameraConfig = cam.RGBDNode->ifconfig.split ( "," );
	if ( cameraConfig.size() > 1 ) {
		uint32_t basePort  = QString ( cameraConfig[1] ).toUInt();
		//if( worker->servers.dfr_servers.count( basePort ) > 0 )
		if( worker->servers.hMaps.count( basePort ) > 0 )
			rgbdParams.talkToBase = true;
		
		uint32_t jointPort = QString ( cameraConfig[0] ).toUInt();
		//if( worker->servers.jm_servers.count( jointPort ) > 0 )
		if( worker->servers.hMaps.count(jointPort) > 0 )
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


void RGBDI::setRegistration ( Registration value, const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	//worker->rgbd_setRegistration ( id, value );
}


Registration RGBDI::getRegistration ( const Ice::Current& )
{
	//return worker->rgbd_getRegistration( id );
	guard gl(worker->innerModel->mutex);
	return RoboCompRGBD::DepthInColor;
}


void RGBDI::getData ( RoboCompRGBD::imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor ::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	ColorSeq color;
	DepthSeq depth;
	PointSeq points;
	this->getImage (color, depth, points, hState, bState );
	
	rgbMatrix.resize (color.size()*3);
	distanceMatrix.resize (color.size());
	for (uint32_t i=0; i<color.size(); i++ ) 
	{
		rgbMatrix[3*i + 0] = color[i].red;
		rgbMatrix[3*i + 1] = color[i].green;
		rgbMatrix[3*i + 2] = color[i].blue;
		distanceMatrix[i] = depth[i];
	}
}


void RGBDI::getDepthInIR ( depthType& distanceMatrix, RoboCompJointMotor ::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	ColorSeq color;
	DepthSeq depth;
	PointSeq points;
	this->getImage( color, depth, points, hState, bState );
	
	distanceMatrix.resize ( 640*480 );
	for ( int i=0; i<640*480; i++ ) {
		distanceMatrix[i] = depth[i];
	}
}


void RGBDI::getImage ( ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor ::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	//worker->rgbd_getImage ( id, color, depth, points, hState, bState );

	IMVCamera &cam = worker->imv->cameras[id];
	QStringList cameraConfig = cam.RGBDNode->ifconfig.split ( "," );
	if (cameraConfig.size() > 1)
	{
		uint32_t basePort  = QString ( cam.RGBDNode->ifconfig.split ( "," ) [1] ).toUInt();
		//std::map<uint32_t, OmniRobotServer>::iterator base;
		//base = worker->servers.omn_servers.find( basePort );
		auto base = worker->servers.hMaps.find( basePort );
		
		bool bstateUpd = false;
		//if (base != worker->servers.omn_servers.end())
		if (base != worker->servers.hMaps.cend())
		{
			try{ //Check if object is type omni or differential
				std::get<OmniRobotServer>(base->second).interface->getBaseState( bState );
				bstateUpd = true;
			}catch(...){/*cout<<"Not omni base"<<std::endl;*/}
		}
		//std::map<uint32_t, DifferentialRobotServer>::iterator baseD;
		//baseD = worker->servers.dfr_servers.find( basePort );
		auto baseD = worker->servers.hMaps.find( basePort );
		//if (baseD != worker->servers.dfr_servers.end())
		if (baseD != worker->servers.hMaps.end())
		{
			try{ //Check if object is type omni or differential
				std::get<DifferentialRobotServer>(baseD->second).interface->getBaseState( bState );
				bstateUpd = true;
			}catch(...){/*cout<<"Not differential base"<<std::endl;*/}
		}
		if (not bstateUpd)
		{
			std::cout<<"Error: no base state updated, basePort "<<basePort<<std::endl;
		}
		uint32_t jointPort = QString ( cam.RGBDNode->ifconfig.split ( "," ) [0] ).toUInt();
		//std::map<uint32_t, JointMotorServer>::iterator joint;
		//joint = worker->servers.jm_servers.find( jointPort );
		auto joint = worker->servers.hMaps.find( jointPort );
		//if ( joint != worker->servers.jm_servers.end() )
		if ( joint != worker->servers.hMaps.cend() )
		{
			RoboCompJointMotor::MotorStateMap newMap;
			std::get<JointMotorServer>(joint->second).interface->getAllMotorState ( newMap );
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
	for ( int i=0; i<height; ++i ) 
	{
		for ( int j=0; j<width; ++j ) 
		{
			const int index  = j + ( i ) *width;
			const int indexI = j + ( height-1-i ) *width;
			color[index].red         = rgb[3*indexI+0];
			color[index].green       = rgb[3*indexI+1];
			color[index].blue        = rgb[3*indexI+2];
			if ( d[indexI] <= 1. ) 
			{
				depth[index] = ( Zn*Zf / ( Zf - d[indexI]* ( Zf-Zn ) ) ) *rndm((td++)%1000001) ;
			} else 
			{
				depth[i] = NAN;
			}
			points[index].x = ( depth[index] * ( ( float ) j- ( width/2. ) ) / focal );
			points[index].y = depth[index] * ( (height/2.) - float(i) ) / focal;
			points[index].z = ( depth[index] );
			points[index].w = 1.;
		}
	}
}


void RGBDI::getDepth ( DepthSeq& depth, RoboCompJointMotor ::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
	ColorSeq color;
	PointSeq points;
	this->getImage ( color, depth, points, hState, bState );
}


void RGBDI::getRGB ( ColorSeq& color, RoboCompJointMotor ::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
	qDebug()<<"get image";
	DepthSeq depth;
	PointSeq points;
	this->getImage ( color, depth, points, hState, bState );
}


void RGBDI::getXYZ ( PointSeq& points, RoboCompJointMotor ::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
	ColorSeq color;
	DepthSeq depth;
	this->getImage ( color, depth, points, hState, bState );
}

