/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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

// Simulator includes
#include "specificworker.h"
#include "specificworker_data.h"

#include <QElapsedTimer>



class RenderThread : public QThread
{
	SpecificWorker* worker;
	
	public:
		explicit RenderThread ( SpecificWorker* worker, QObject* parent = 0 ) : QThread(parent) {
			this->worker = worker;
		}
	void run() {
		usleep( 1000*1000 );
		forever {
			worker->compute();
			usleep( 10*1000 );
		}
	}
};



// ------------------------------------------------------------------------------------------------
// Specific worker
// ------------------------------------------------------------------------------------------------

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker ( MapPrx& _mprx, Ice::CommunicatorPtr _communicator, const char* _innerModelXML ) : GenericWorker ( _mprx )
{
	d = new Data;
	
	// Create the server handlers
	d->worker = this;
	d->communicator = _communicator;
	d->laserDataCartArray.clear();
	
	// Initialize Inner model
	d->innerModel = new IM2::InnerModel( _innerModelXML );
	
	// Initialize the Inner Model Viewer
	QGLFormat fmt;
	QGLFormat::setDefaultFormat ( fmt );
	fmt.setDoubleBuffer ( true );
	fmt.setSampleBuffers( true );
	d->viewer = new OsgView ( frameOSG );
	d->imv = new IM2::Viewer ( d->innerModel, "root", d->viewer->getRootGroup() );
	d->manipulator = new osgGA::TrackballManipulator();
	d->viewer->setCameraManipulator ( d->manipulator );
	d->viewer->setFormat(fmt);
	
	d->innerModel->initPhysics( d->imv );
	
	// Resize the viewport to 720p
	frameOSG->setMinimumSize( 1280, 720 );
	frameOSG->setMaximumSize( 1280, 720 );
	
	// Set the initial camera position
	d->manipulator->setCenter( osg::Vec3d(0.0,0.8,0.0) );
	d->manipulator->setDistance( 2.1 );
	d->manipulator->setRotation( osg::Quat(-M_PI/6.0, osg::Vec3f(1,0,0) ) );
	
	// Connect all the signals
	connect ( topView,   SIGNAL ( clicked() ), this, SLOT ( setTopPOV() ) );
	connect ( leftView,  SIGNAL ( clicked() ), this, SLOT ( setLeftPOV() ) );
	connect ( rightView, SIGNAL ( clicked() ), this, SLOT ( setRightPOV() ) );
	connect ( frontView, SIGNAL ( clicked() ), this, SLOT ( setFrontPOV() ) );
	connect ( backView,  SIGNAL ( clicked() ), this, SLOT ( setBackPOV() ) );
	connect ( actionObject, SIGNAL ( triggered() ), this, SLOT ( objectTriggered() ) );
	connect ( actionVisual, SIGNAL ( triggered() ), this, SLOT ( visualTriggered() ) );
	
	// Initialization
	objectTriggered();
	visualTriggered();
	setPeriod ( 16 );
	
// 	RenderThread* rt = new RenderThread( this );
// 	rt->start();
}


/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}


IM2::InnerModel *SpecificWorker::getInnerModel() const
{
	return d->innerModel;
}


void SpecificWorker::startServers()
{
	d->walkTree();
}



// ------------------------------------------------------------------------------------------------
// Slots
// ------------------------------------------------------------------------------------------------

void SpecificWorker::compute( )
{
	// Compute the elapsed time interval since the last update
// 	static QElapsedTimer counter;
// 	const float elapsed = float( counter.restart() ) / 1000.0f;
	
	static QElapsedTimer counter;
	static qint64 lastTime = counter.nsecsElapsed();
	const qint64 currentTime = counter.nsecsElapsed();
	const double elapsed = double( currentTime - lastTime ) / 1e9;
	lastTime = currentTime;
	
	// HACK
	if( true ) {
		JointMovement m1;
		m1.endPos = 0.5f;
		m1.endSpeed = 0.1f;
		m1.maxAcc = INFINITY;
		m1.mode = JointMovement::TargetPosition;
		d->jointMovements["joint_left_shoulder_1"] = m1;
		
		JointMovement m2;
		m2.endPos = 1.5f;
		m2.endSpeed = 0.3f;
		m2.maxAcc = INFINITY;
		m2.mode = JointMovement::TargetPosition;
		d->jointMovements["joint_left_shoulder_2"] = m2;
		
// 		JointMovement m3;
// 		m3.endPos = -1.0f;
// 		m3.endSpeed = 0.1f;
// 		m3.maxAcc = INFINITY;
// 		m3.mode = JointMovement::TargetPosition;
// 		d->jointMovements["joint_left_shoulder_3"] = m3;
		
		JointMovement m4;
		m4.endPos = 1.0f;
		m4.endSpeed = 0.2f;
		m4.maxAcc = INFINITY;
		m4.mode = JointMovement::TargetPosition;
		d->jointMovements["joint_left_elbow"] = m4;
	}
	
	// Update the world
	QMutexLocker locker ( mutex );
	
	// Remove previous laser shapes
	for ( QHash<QString, IM2::Laser*>::iterator laser = d->imv->lasers.begin(); laser != d->imv->lasers.end(); ++laser ) {
		if ( (*laser)->osg_node->getNumChildren() > 0 ) {
			(*laser)->osg_node->removeChild ( 0, (*laser)->osg_node->getNumChildren() );
		}
	}
	
	// Camera render
	QHash<QString, IM2::Camera*>::const_iterator camera;
	for ( camera = d->imv->cameras.constBegin() ; camera != d->imv->cameras.constEnd() ; ++camera ) {
		RTMat rt= d->innerModel->getTransformationMatrix ( "root", camera.key() );
		
		// Put camera in its position and render its frame
		(*camera)->osg_camera->getCameraManipulator()->setByMatrix ( IM2::QMatToOSGMat4 ( rt ) );
		(*camera)->osg_camera->frame();
	}
	
	// Laser rendering
	for ( QHash<QString, IM2::Laser*>::iterator laser = d->imv->lasers.begin(); laser != d->imv->lasers.end(); ++laser ) {
		QString id=(*laser)->id;

		if ( d->laserDataCartArray.contains ( id ) ==false ) {
			//laserDataCartArray.insert(id);
			osg::Vec3Array *v= new osg::Vec3Array();
			v->resize ( (*laser)->im_measures+1 );
			d->laserDataCartArray.insert ( id,v );
		}

		// create and insert laser data
		d->laserDataArray.insert ( (*laser)->id, d->LASER_createLaserData ( **laser ) );
		
		// create and insert laser shape
		osg::ref_ptr<osg::Node> p=NULL;
		if ( id=="laserSecurity" ) {
			p = d->viewer->addPolygon ( * ( d->laserDataCartArray[id] ), osg::Vec4 ( 0.,0.,1.,0.4 ) );
		} else {
			p = d->viewer->addPolygon ( * ( d->laserDataCartArray[id] ) );
		}
		if ( p!=NULL ) {
			(*laser)->osg_node->addChild ( p );
		}
	}
	
	// Update joints and compute physic interactions
// 	printf( "Updating physics: %f\n", elapsed*1000.0f );
	const float timescale = 0.5f;
	d->updateJoints( elapsed * timescale );
	d->innerModel->cleanupTables();
	d->innerModel->updatePhysics( elapsed * timescale );
	
	// Shutdown empty servers
	for (int i=0; i<d->jointServersToShutDown.size(); i++) {
		d->jointServersToShutDown[i]->shutdown();
	}
	d->jointServersToShutDown.clear();
	
	// Resize world widget if necessary, and render the world
	if ( d->viewer->size() != frameOSG->size() ) {
		d->viewer->setFixedSize ( frameOSG->width(), frameOSG->height() );
	}
	d->viewer->update();
	
// 	// Save the frame sequence
// 	static int frame = 0;
// 	if( (frame>0) && (frame<=300) ) {
// 		char foo[100];
// 		snprintf( foo, 100, "%05d.bmp", frame-1 );
// 		d->viewer->grabFrameBuffer().save( foo );
// 	}
// 	frame++;
}


void SpecificWorker::objectTriggered()
{
	if ( actionObject->isChecked() ) {
		OBJECTWidget->show();
	} else {
		OBJECTWidget->hide();
	}
}


void SpecificWorker::visualTriggered()
{
	if ( actionVisual->isChecked() ) {
		VISUALWidget->show();
	} else {
		VISUALWidget->hide();
	}
}


void SpecificWorker::setTopPOV()
{
	setMainCamera ( d->manipulator, IM2::TOP_POV );
}


void SpecificWorker::setFrontPOV()
{
	setMainCamera ( d->manipulator, IM2::FRONT_POV );
}


void SpecificWorker::setBackPOV()
{
	setMainCamera ( d->manipulator, IM2::BACK_POV );
}


void SpecificWorker::setLeftPOV()
{
	setMainCamera ( d->manipulator, IM2::LEFT_POV );
}


void SpecificWorker::setRightPOV()
{
	setMainCamera ( d->manipulator, IM2::RIGHT_POV );
}


void SpecificWorker::closeEvent ( QCloseEvent *event )
{
	event->accept();
	exit ( EXIT_SUCCESS );
}



// ------------------------------------------------------------------------------------------------
// Camera.ice
// ------------------------------------------------------------------------------------------------

void SpecificWorker::cam_camera_getYUVImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{
}


void SpecificWorker::cam_getYImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


void SpecificWorker::cam_getYLogPolarImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


void SpecificWorker::cam_getYImageCR ( const QString& server, int cam, int div, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


void SpecificWorker::cam_getRGBPackedImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


void SpecificWorker::cam_getYRGBImage ( const QString& server, int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState )
{}


TCamParams SpecificWorker::cam_getCamParams ( const QString& server )
{
	TCamParams tcp;
	return tcp;
}


void SpecificWorker::cam_setInnerImage ( const QString& server, const RoboCompCamera::imgType& roi )
{}



// ------------------------------------------------------------------------------------------------
// CommonBehavior.ice
// ------------------------------------------------------------------------------------------------



// ------------------------------------------------------------------------------------------------
// DifferentialRobot.ice
// ------------------------------------------------------------------------------------------------

void SpecificWorker::dfr_getBaseState ( const QString& server, TBaseState& state )
{}


void SpecificWorker::dfr_getBasePose ( const QString& server, int& x, int& z, float& alpha )
{}


void SpecificWorker::dfr_setSpeedBase ( const QString& server, float adv, float rot )
{}


void SpecificWorker::dfr_stopBase ( const QString& server )
{}


void SpecificWorker::dfr_resetOdometer ( const QString& server )
{}


void SpecificWorker::dfr_setOdometer ( const QString& server, const TBaseState& state )
{}


void SpecificWorker::dfr_setOdometerPose ( const QString& server, int x, int z, float alpha )
{}


void SpecificWorker::dfr_correctOdometer ( const QString& server, int x, int z, float alpha )
{}



// ------------------------------------------------------------------------------------------------
// InnerModelManager.ice
// ------------------------------------------------------------------------------------------------

// Moves item to the position defined by pose respect to the base
bool SpecificWorker::imm_setPose ( const QString& server, const std::string& base, const std::string& item, const RoboCompInnerModelManager::Pose3D& pose )
{
	QMutexLocker locker ( mutex );

	QString qBase = QString::fromStdString ( base );
	QString qItem = QString::fromStdString ( item );
	QString m="RoboCompInnerModelManager::setPose()";

	//check type transform
	IM2::GenericJoint *aux = dynamic_cast<IM2::GenericJoint*> ( d->getNode ( qBase, m ) );
	d->checkOperationInvalidNode ( aux, m + qBase +"can't be use as base because it's not a IM2::GenericJoint node." );
	aux = NULL;
	aux = dynamic_cast<IM2::GenericJoint*> ( d->getNode ( qItem, m ) );
	d->checkOperationInvalidNode ( aux, m + qItem +"can't be use as item because it's not a IM2::GenericJoint node." );

	///T[bi]=T[bp]*T[pi] ; (T[bp]-1)*T[bi]=T[pi]
	///T[bi]
	RTMat Tbi;
	Tbi.setR ( pose.rx,pose.ry,pose.rz );
	Tbi.setTr ( pose.x,pose.y,pose.z );

	///Tbp Inverse = Tpb. This gets Tpb directly. It's the same
	RTMat Tpb= d->innerModel->getTransformationMatrix ( d->innerModel->getNode ( qItem )->im_parent->id, qBase );
	///New Tpi
	RTMat Tpi=Tpb*Tbi;

	QVec angles =Tpi.extractAnglesR();
	QVec tr=Tpi.getTr();

#ifdef INNERMODELMANAGERDEBUG
	qDebug() <<"parent of "<<qItem<<innerModel->getNode ( qItem )->parent->id;
	qDebug() <<"T[pb]";
	Tpb.getTr().print ( "Translation" );
	Tpb.extractAnglesR().print ( "angles" );
	qDebug() <<"---";
	qDebug() <<"T[bi]";
	Tbi.getTr().print ( "Translation" );
	Tbi.extractAnglesR().print ( "angles" );
	qDebug() <<"---";
	qDebug() <<"T[pi]";
	tr.print ( "translation" );
	angles.print ( "angles" );
	qDebug() <<"---";
#endif

	d->innerModel->updateGenericJointValues ( qItem,tr.x(),tr.y(),tr.z(),angles.x(),angles.y(),angles.z() );

// 	if ( collisiondetection->isChecked() ) {
// 		checkPoseCollision ( qItem,m );
// 	}

	return true;
}


bool SpecificWorker::imm_setPoseFromParent ( const QString& server, const std::string& item, const RoboCompInnerModelManager::Pose3D& pose )
{
	QMutexLocker locker ( mutex );
	QString qItem = QString::fromStdString ( item );
	QString m = "RoboCompInnerModelManager::setPoseFromParent()";

	IM2::GenericJoint *aux = dynamic_cast<IM2::GenericJoint*> ( d->getNode ( QString::fromStdString ( item ),m ) );
	d->checkOperationInvalidNode ( aux, m + qItem +"can't be use as item because it's not a IM2::GenericJoint node." );

	d->innerModel->updateGenericJointValues ( qItem,pose.x, pose.y, pose.z, pose.rx , pose.ry, pose.rz );
	d->innerModel->cleanupTables();

// 	if ( collisiondetection->isChecked() ) {
// 		checkPoseCollision ( qItem,m );
// 	}

	return true;
}


/// ---------------------------------------------------------------------------------------
// Provides the pose of a certain item respect to the base.
/// ---------------------------------------------------------------------------------------
bool SpecificWorker::imm_getPose ( const QString& server, const std::string& item, const std::string& base, RoboCompInnerModelManager::Pose3D& pose )
{
	QMutexLocker locker ( mutex );
	QVec p;
	QString qBase = QString::fromStdString ( base );
	QString qItem = QString::fromStdString ( item );
	QString m="RoboCompInnerModelManager::getPose()";

	//check type transform
	IM2::GenericJoint *aux = dynamic_cast<IM2::GenericJoint*> ( d->getNode ( qBase, m ) );
	d->checkOperationInvalidNode ( aux, m + qBase +"can't be use as base because it's not a IM2::GenericJoint node." );
	aux = NULL;
	aux = dynamic_cast<IM2::GenericJoint*> ( d->getNode ( qItem, m ) );
	d->checkOperationInvalidNode ( aux, m + qItem +"can't be use as item because it's not a IM2::GenericJoint node." );

	// calculate position
	p = d->innerModel->transform ( QString::fromUtf8 ( base.c_str() ), QVec::vec3 ( 0,0,0 ),QString::fromUtf8 ( item.c_str() ) );
	pose.x = p.x();
	pose.y = p.y();
	pose.z = p.z();
	//calulate rotation
	p = d->innerModel->getRotationMatrixTo ( QString::fromUtf8 ( base.c_str() ), QString::fromUtf8 ( item.c_str() ) ).extractAnglesR();
	pose.rx = p.x();
	pose.ry = p.y();
	pose.rz = p.z();

	return true;
}


// Provides the pose of a certain item respect to the parent
bool SpecificWorker::imm_getPoseFromParent ( const QString& server, const std::string& item, RoboCompInnerModelManager::Pose3D& pose )
{
	QMutexLocker locker ( mutex );
	QString m="RoboCompInnerModelManager::getPoseFromParent()";

	IM2::GenericJoint *aux = dynamic_cast<IM2::GenericJoint*> ( d->getNode ( QString::fromStdString ( item ), m ) );
	d->checkOperationInvalidNode ( aux,m + aux->id +"can't be use as base because it's not a IM2::GenericJoint node." );

	pose.x = aux->im_tx;
	pose.y = aux->im_ty;
	pose.z = aux->im_tz;
	pose.rx = aux->im_rx;
	pose.ry = aux->im_ry;
	pose.rz = aux->im_rz;
	
	return true;
}


// Provides the transform of a certain point expressed in Base to Item.
bool SpecificWorker::imm_transform ( const QString& server, const std::string& base, const std::string& item, const RoboCompInnerModelManager::coord3D& coordInItem, RoboCompInnerModelManager::coord3D& coordInBase )
{
	QMutexLocker locker ( mutex );
	QVec p;
	QString qBase = QString::fromStdString ( base );
	QString qItem = QString::fromStdString ( item );
	QString m="RoboCompInnerModelManager::transform()";

	//check type transform
	IM2::GenericJoint *aux = dynamic_cast<IM2::GenericJoint*> ( d->getNode ( qBase, m ) );
	d->checkOperationInvalidNode ( aux, m + qBase +"can't be use as base because it's not a IM2::GenericJoint node." );

	aux = NULL;
	aux = dynamic_cast<IM2::GenericJoint*> ( d->getNode ( qItem, m ) );
	d->checkOperationInvalidNode ( aux, m + qItem +"can't be use as item because it's not a IM2::GenericJoint node." );

	// calculate position
	p = d->innerModel->transform ( QString::fromUtf8 ( base.c_str() ), QVec::vec3 ( coordInItem.x,coordInItem.y,coordInItem.z ),QString::fromUtf8 ( item.c_str() ) );
	coordInBase.x = p.x();
	coordInBase.y = p.y();
	coordInBase.z = p.z();

	return true;
}


RoboCompInnerModelManager::Matrix SpecificWorker::imm_getTransformationMatrix(const std::string& item, const std::string& base)
{
// 	const QString qBase = QString::fromStdString(base);
// 	const QString qItem = QString::fromStdString(item);
// 	const QString m="RoboCompInnerModelManager::transform()";
// 	QMutexLocker locker(mutex);
// 
// 	//check type transform
// 	IM2::Transform *aux = dynamic_cast<InnerModelTransform*>(d->getNode(qBase, m));
// 	d->checkOperationInvalidNode(aux, m + qBase +"can't be used as base because it's not a InnerModelTransform node.");
// 
// 	aux = NULL;
// 	aux = dynamic_cast<InnerModelTransform*>(d->getNode(qItem, m));
// 	d->checkOperationInvalidNode(aux, m + qItem +"can't be used as item because it's not a InnerModelTransform node.");
// 
// 	// calculate position
// 	RTMat retA = d->innerModel->getTransformationMatrix(qBase, qItem);
	RoboCompInnerModelManager::Matrix retB;
// 
// 	retB.cols = retA.nCols();
// 	retB.rows = retA.nRows();
// 	retB.data.resize(retB.cols*retB.rows);
// 	for (int r=0; r<retB.rows; r++)
// 	{
// 		for (int c=0; c<retB.cols; c++)
// 		{
// 			retB.data[r*retB.cols + c] = retA(r, c);
// 		}
// 	}
// 
	return retB;
}


// Scales a mesh to a new size.
bool SpecificWorker::imm_setScale ( const QString& server, const std::string& item, float scaleX, float scaleY, float scaleZ )
{
#ifndef USE_BULLET
	QMutexLocker locker ( mutex );
	QString qItem = QString::fromStdString ( item );
	QString m="RoboCompInnerModelManager::setScale()";

	IM2::Mesh *aux = dynamic_cast<IM2::Mesh*> ( d->getNode ( QString::fromStdString ( item ),m ) );
	d->checkOperationInvalidNode ( aux,m + qItem +"can't be used because it's not a InnerModelMesh node." );

	aux->setScale ( scaleX, scaleY, scaleZ );
	d->innerModel->update();

#ifdef INNERMODELMANAGERDEBUG
	try {
		checkPoseCollision ( qItem,m );
	} catch ( RoboCompInnerModelManager::InnerModelManagerError err ) {
		std::cout<<err.what() <<" "<<err.text<< "\n";
		std::cout<< "\n";
		///come back to t= (t+1) -t

		//to check => maybe using a tag in the xml (collide="true"  ) to decide if allow collitions or not
		//  innerModel->updateTransformValues(qItem,p.x, p.y, p.z, p.rx , p.ry, p.rz);
		//  innerModel->update();
		throw err;
	}
#endif
#endif

	return true;
}


bool SpecificWorker::imm_setPlane ( const QString& server, const std::string& item, const RoboCompInnerModelManager::Plane3D& plane )
{
	QMutexLocker locker ( mutex );
	QString m="RoboCompInnerModelManager::setPlane()";
	IM2::Plane *aux = dynamic_cast<IM2::Plane*> ( d->getNode ( QString::fromStdString ( item ), m ) );
	d->checkOperationInvalidNode ( aux,m + aux->id +"can't be use as base because it's not of the type IM2::Plane." );
	d->innerModel->updatePlaneValues ( QString::fromStdString ( item ), plane.nx, plane.ny, plane.nz, plane.px, plane.py, plane.pz );
	return true;
}


// void SpecificWorker::addPointCloud(const std::string &id)
// {
// }


void SpecificWorker::imm_setPointCloudData ( const QString& server, const std::string& id, const RoboCompInnerModelManager::PointCloudVector& cloud )
{
	QString m = QString ( "SpecificWorker::setPointCloudData" );

	std::cout<<"setPointCloudData: "<<id<<" "<<cloud.size() <<std::endl;

	//IM2::IMVPointCloud *pcNode = d->imv->clouds[QString::fromStdString ( id )];
	IM2::PointCloud* pcNode = dynamic_cast<IM2::PointCloud*>( d->innerModel->getNode(QString::fromStdString( id )) );

	int np = cloud.size();
	osg::Vec3Array* coords = pcNode->osg_points;
	osg::Vec4Array* colors = pcNode->osg_colors;
	coords->resize( np );
	colors->resize( np );
	for ( int i=0 ; i<np; i++ ) {
		(*coords)[i] = IM2::QVecToOSGVec ( QVec::vec3 ( cloud[i].x, -cloud[i].y, cloud[i].z ) );
		(*colors)[i] = osg::Vec4f ( float ( cloud[i].r ) /255, float ( cloud[i].g ) /255, float ( cloud[i].b ) /255, 1.f );
	}
	
	pcNode->setPointSize ( 1.0f );
	pcNode->reload();
	//d->imv->update();
}


bool SpecificWorker::imm_addTransform ( const QString& server, const std::string& item, const std::string& engine, const std::string& base, const RoboCompInnerModelManager::Pose3D& pose )
{
// 	QMutexLocker locker ( mutex );
// 
// 	IM2::Node *parent = d->getNode ( QString::fromStdString ( base ), "RoboCompInnerModelManager::addTransform()" );
// 	d->checkNodeAlreadyExists ( QString::fromStdString ( item ), "RoboCompInnerModelManager::addTransform()" );
// 	
// 	QString qEngine = QString::fromStdString( engine );
// 	if ( qEngine !="static" and qEngine !="bullet" ) {
// 		qEngine = "static";
// 	}
// 	qDebug()<<"engine"<<qEngine;
// 	IM2::GenericJoint *tr = d->innerModel->newTransform ( QString::fromStdString ( item ), QString::fromStdString ( "static" ) ,parent, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz );
// 
// 	parent->addChild ( tr );
// 	d->imv->recursiveConstructor ( tr, d->imv->transformHash[parent->id] ); // imv->osgmeshes,imv->osgmeshPats);
// 	d->imv->update();
// #ifdef INNERMODELMANAGERDEBUG
// 	qDebug() <<"transform: pose.x<<pose.y<<pose.z"<<pose.x<<pose.y<<pose.z<<QString::fromStdString ( item );
// #endif
// 	
	return true;
}


bool SpecificWorker::imm_addJoint ( const QString& server, const std::string& item, const std::string& base, const RoboCompInnerModelManager::jointType& j )
{
// 	QMutexLocker locker ( mutex );
// 	RoboCompInnerModelManager::Pose3D pose = j.pose;
// 
// 	IM2::GenericJoint *parent=dynamic_cast<IM2::GenericJoint *> ( d->getNode ( QString::fromStdString ( base ), "RoboCompInnerModelManager::addJoint()" ) );
// 	d->checkNodeAlreadyExists ( QString::fromStdString ( item ), "RoboCompInnerModelManager::addJoint()" );
// 
// 	InnerModelJoint *jN = d->innerModel->newJoint ( QString::fromStdString ( item ), parent,
// 	                      j.lx, j.ly, j.lz, j.hx, j.hy, j.hz,
// 	                      pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, j.min, j.max, j.port, j.axis );
// 	parent->addChild (jN);
// 
// 	// Create Interface in case the port is not 0
// 	if (jN->port != 0)
// 		d->addJM(jN);
// 
// 	d->imv->recursiveConstructor ( jN, d->imv->mts[parent->id], d->imv->mts, d->imv->meshHash ); // imv->osgmeshes,imv->osgmeshPats);

	return true;
}


bool SpecificWorker::imm_addMesh ( const QString& server, const std::string& item, const std::string& base, const RoboCompInnerModelManager::meshType& m )
{
#ifndef USE_BULLET
	QMutexLocker locker ( mutex );
	QString msg="RoboCompInnerModelManager::addMesh()";
#ifdef INNERMODELMANAGERDEBUG
	qDebug() <<msg<<QString::fromStdString ( base ) <<QString::fromStdString ( item );
	qDebug() <<QString::fromStdString ( m.meshPath );
#endif
	IM2::GenericJoint *parent = dynamic_cast<IM2::GenericJoint*> ( d->getNode ( QString::fromStdString ( base ), msg ) );

	//Checking if its parent is not a mesh.
	d->checkOperationInvalidNode ( parent, msg );
	d->checkNodeAlreadyExists ( QString::fromStdString ( item ), msg );
	d->checkInvalidMeshValues ( m,msg );

	int render = m.render;
	if ( render!=0 && render!=1 ) {
		render=0;
	}
	
	InnerModelMesh *mesh = d->innerModel->newMesh (
		QString::fromStdString ( item ),
		parent,
		QString::fromStdString ( m.meshPath ),
		m.scaleX, m.scaleY, m.scaleZ,
		render,
		m.pose.x, m.pose.y, m.pose.z,
		m.pose.rx, m.pose.ry, m.pose.rz );

	mesh->setScale ( m.scaleX, m.scaleY, m.scaleZ );
	parent->addChild ( mesh );

	d->imv->recursiveConstructor ( mesh, d->imv->mts[parent->id], d->imv->mts, d->imv->meshHash ); // osgmeshes,imv->osgmeshPats);
	d->imv->update();
#endif
	return true;
}


bool SpecificWorker::imm_addPlane ( const QString& server, const std::string& item, const std::string& base, const RoboCompInnerModelManager::Plane3D& p )
{
// 	QMutexLocker locker ( mutex );
// 
// 	IM2::Node *parent = d->getNode ( QString::fromStdString ( base ), "RoboCompInnerModelManager::addPlane()" );
// 	d->checkNodeAlreadyExists ( QString::fromStdString ( item ), "RoboCompInnerModelManager::addPlane()" );
// 
// 
// 	IM2::Plane *plane = d->innerModel->newPlane ( QString::fromStdString ( item ), parent, QString::fromStdString ( p.texture ),
// 	                         p.width, p.height, p.thickness, 1,
// 	                         p.nx, p.ny, p.nz, p.px, p.py, p.pz );
// 	parent->addChild ( plane );
// 
// 	d->imv->recursiveConstructor ( plane, d->imv->mts[parent->id], d->imv->mts, d->imv->meshHash );
// 	d->imv->update();

	return true;
}


bool SpecificWorker::imm_addAttribute ( const QString& server, const std::string& idNode, const std::string& name, const std::string& type, const std::string& value )
{
	QMutexLocker locker ( mutex );
	QString qIdNode=QString::fromStdString ( idNode );
	QString qName=QString::fromStdString ( name );
	QString qType=QString::fromStdString ( type );
	QString qValue=QString::fromStdString ( value );
	QString m="RoboCompInnerModelManager::addAttribute()";

	IM2::Node *node = d->getNode ( qIdNode, m );
	d->AttributeAlreadyExists ( node,qName,m );

	IM2::AttributeType t;
	t.type=qType;
	t.value=qValue;
	node->im_attributes.insert ( qName,t );

	return true;
}


bool SpecificWorker::imm_setAttribute ( const QString& server, const std::string& idNode, const std::string& name, const std::string& type, const std::string& value )
{
	QMutexLocker locker ( mutex );
	QString qIdNode=QString::fromStdString ( idNode );
	QString qName=QString::fromStdString ( name );
	QString qType=QString::fromStdString ( type );
	QString qValue=QString::fromStdString ( value );

	QString m="RoboCompInnerModelManager::setAttribute()";

	IM2::Node *node = d->getNode ( qIdNode, m );
	d->NonExistingAttribute ( node,qName,m );

	node->im_attributes[qName].type=qType;
	node->im_attributes[qName].value=qValue;

	return true;
}


bool SpecificWorker::imm_getAttribute ( const QString& server, const std::string& idNode, const std::string& name, std::string& type, std::string& value )
{
	QMutexLocker locker ( mutex );
	QString qIdNode=QString::fromStdString ( idNode );
	QString qName=QString::fromStdString ( name );
	QString m="RoboCompInnerModelManager::getAttribute()";

	IM2::Node *node = d->getNode ( qIdNode, m );
	d->NonExistingAttribute ( node, qName,m );

	type= node->im_attributes[qName].type.toStdString();
	value=node->im_attributes[qName].value.toStdString();

	return true;
}


bool SpecificWorker::imm_removeAttribute ( const QString& server, const std::string& idNode, const std::string& name )
{
	QMutexLocker locker ( mutex );
	QString qIdNode=QString::fromStdString ( idNode );
	QString qName=QString::fromStdString ( name );
	QString m="RoboCompInnerModelManager::removeAttribute()";

	IM2::Node *node = d->getNode ( qIdNode, m );
	d->NonExistingAttribute ( node, qName,m );

	node->im_attributes.remove ( qName );

	return true;
}


bool SpecificWorker::imm_removeNode ( const QString& server, const std::string& item )
{
	QMutexLocker locker ( mutex );
	QString msg="RoboCompInnerModelManager::removeNode()";
	
	#ifdef INNERMODELMANAGERDEBUG
	qDebug() << msg << QString::fromStdString ( item );
	#endif

	// Check that we are not trying to delete the whole world!
	QString id =QString::fromStdString ( item );
	if ( id=="world" || id=="root" ) {
		#ifdef INNERMODELMANAGERDEBUG
		qDebug() <<msg<<id<<"Can't remove root elements";
		#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::OperationInvalidNode;
		std::ostringstream oss;
		oss << msg.toStdString() <<" cannot remove node: " <<id.toStdString();
		err.text = oss.str();
		throw err;
	}

	// Find the node and its children
	IM2::Node *node = d->getNode ( QString::fromStdString ( item ), msg );
	d->checkOperationInvalidNode ( node, msg );
	QStringList childrenNames;
	d->innerModel->getSubTree( node, childrenNames );

	// Delete the node and remove the handlers
	foreach ( QString name, childrenNames ) {
		// If the node is a joint, remove the handler
		IM2::HingeJoint * joint = dynamic_cast<IM2::HingeJoint *> (d->innerModel->getNode(name) );
		if ( (joint != NULL) && (joint->im_port != 0) ) {
			d->removeJM( joint );
		}
	}
	d->innerModel->removeSubTree ( node );
	
	// Remove the node and its children from the viewer
// 	d->imv->removeNode( node->id );
// 	foreach ( QString n, childrenNames ) {
// 		d->imv->removeNode( n );
// 	}

	return true;
}


void SpecificWorker::imm_getAllNodeInformation ( const QString& server, RoboCompInnerModelManager::NodeInformationSequence& nodesInfo )
{
	nodesInfo.clear();
	d->getRecursiveNodeInformation ( nodesInfo, d->innerModel->getRoot() );
}



// ------------------------------------------------------------------------------------------------
// IMU.ice
// ------------------------------------------------------------------------------------------------

void SpecificWorker::imu_updateIMUData ( const QString& server, QString id )
{
	QMutexLocker locker ( mutex );

	QMat R = d->innerModel->getRotationMatrixTo ( id, "root" );
	QVec acc   = R * QVec::vec3 ( 0.0, -9.80665,  0.0 );
	QVec north = R * QVec::vec3 ( 0.0,      0.0, 10.0 );

	d->data_imu.acc.XAcc = acc ( 0 );
	d->data_imu.acc.YAcc = acc ( 1 );
	d->data_imu.acc.ZAcc = acc ( 2 );

	d->data_imu.rot.Pitch = atan2 ( acc ( 2 ) ,  -acc ( 1 ) );
	d->data_imu.rot.Roll  = atan2 ( acc ( 0 ) ,  -acc ( 1 ) );
	d->data_imu.rot.Yaw   = atan2 ( north ( 0 ), north ( 2 ) );

	d->data_imu.temperature = 25.;
}


DataImu SpecificWorker::imu_getDataImu ( const QString& server )
{
	return d->data_imu;
}


void SpecificWorker::imu_resetImu ( const QString& server )
{
}



// ------------------------------------------------------------------------------------------------
// JointMotor.ice
// ------------------------------------------------------------------------------------------------

void SpecificWorker::jm_setPosition ( const QString& name, const MotorGoalPosition& goal )
{
	if ( goal.maxSpeed == 0.0f ) {
		printf ( "Instantaneous movement!\n" );
		JointMovement m;
		m.endPos = goal.position;
		m.endSpeed = goal.maxSpeed;
		m.maxAcc = INFINITY;
		m.mode = JointMovement::FixedPosition;
		d->jointMovements[name] = m;
	} else {
		printf ( "Target position\n" );
		JointMovement m;
		m.endPos = goal.position;
		m.endSpeed = goal.maxSpeed;
		m.maxAcc = INFINITY;
		m.mode = JointMovement::TargetPosition;
		d->jointMovements[name] = m;
	}
}
// 		joint->setAngle ( goal.position );

void SpecificWorker::jm_setVelocity ( const QString& name, const MotorGoalVelocity& goal )
{
	printf ( "Target speed\n" );
	JointMovement m;
	m.endPos = INFINITY;
	m.endSpeed = goal.velocity;
	m.maxAcc = goal.maxAcc;
	m.mode = JointMovement::TargetSpeed;
	d->jointMovements[name] = m;
}


void SpecificWorker::jm_setSyncPosition ( const QString& server, const MotorGoalPositionList& listGoals )
{}


void SpecificWorker::jm_setSyncVelocity ( const QString& server, const MotorGoalVelocityList& listGoals )
{}


MotorParams SpecificWorker::jm_getMotorParams ( const QString& server, const std::string& motor )
{
	MotorParams mp;
	return mp;
}


MotorState SpecificWorker::jm_getMotorState ( const QString& server, const std::string& motor )
{
	MotorState ms;
	return ms;
}


MotorStateMap SpecificWorker::jm_getMotorStateMap ( const QString& server, const MotorList& mList )
{
	MotorStateMap msm;
	return msm;
}


void SpecificWorker::jm_getAllMotorState ( const QString& server, MotorStateMap& mstateMap )
{}


MotorParamsList SpecificWorker::jm_getAllMotorParams ( const QString& server )
{
	MotorParamsList mpl;
	return mpl;
}


RoboCompJointMotor::BusParams SpecificWorker::jm_getBusParams ( const QString& server )
{
	BusParams bp;
	return bp;
}


void SpecificWorker::jm_setZeroPos ( const QString& server, const std::string& motor )
{}


void SpecificWorker::jm_setSyncZeroPos ( const QString& server )
{}


void SpecificWorker::jm_stopAllMotors ( const QString& server )
{}


void SpecificWorker::jm_stopMotor ( const QString& server, const std::string& motor )
{}


void SpecificWorker::jm_releaseBrakeAllMotors ( const QString& server )
{}


void SpecificWorker::jm_releaseBrakeMotor ( const QString& server, const std::string& motor )
{}


void SpecificWorker::jm_enableBrakeAllMotors ( const QString& server )
{}


void SpecificWorker::jm_enableBrakeMotor ( const QString& server, const std::string& motor )
{}



// ------------------------------------------------------------------------------------------------
// Laser.ice
// ------------------------------------------------------------------------------------------------

// TLaserData SpecificWorker::laser_getLaserData ( const QString& server )
// {
// 	QMutexLocker l ( mutex );
// 	
// 	Laser &las = d->imv->lasers[server];
// 	QString laserConfig = las.laserNode->ifconfig;
// 	uint32_t basePort  = laserConfig.toUInt();
// 	RoboCompDifferentialRobot::TBaseState bState;
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


TLaserData SpecificWorker::laser_getLaserAndBStateData ( const QString& server, RoboCompDifferentialRobot::TBaseState& state )
{
	QMutexLocker l ( mutex );
	
	IM2::Laser* las = dynamic_cast<IM2::Laser*>( d->innerModel->getNode(server) );
	
	QString laserConfig = las->im_ifconfig;
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



// ------------------------------------------------------------------------------------------------
// RGBD.ice
// ------------------------------------------------------------------------------------------------

// 	throw std::string( __PRETTY_FUNCTION__ ) + std::string( " not implemented yet!" );

TRGBDParams SpecificWorker::rgbd_getRGBDParams ( const QString& server )
{
	QMutexLocker locker ( mutex );
	
	IM2::Camera* cam = dynamic_cast<IM2::Camera*>( d->innerModel->getNode(server) );

	RoboCompRGBD::TRGBDParams rgbdParams;
	rgbdParams.driver = "RCIS";
	rgbdParams.device = server.toStdString();
	rgbdParams.timerPeriod = 33;//timer.interval();

	QStringList cameraConfig = cam->im_ifconfig.split ( "," );
	if ( cameraConfig.size() > 1 ) {
		uint32_t basePort  = QString ( cameraConfig[1] ).toUInt();
		if( d->dfr_servers.count( basePort ) > 0 )
			rgbdParams.talkToBase = true;
		
		uint32_t jointPort = QString ( cameraConfig[0] ).toUInt();
		if( d->jm_servers.count( jointPort ) > 0 )
			rgbdParams.talkToJointMotor = true;
	}

	RoboCompRGBD::CameraParameters camParams;
	const IM2::CameraParams p = cam->getParams();
	camParams.focal = p.focal;
	camParams.width = p.width;
	camParams.height = p.height;
	camParams.size = p.width*p.height;
	camParams.FPS = rgbdParams.timerPeriod;
	rgbdParams.color = camParams;
	rgbdParams.depth = camParams;

	return rgbdParams;
}


void SpecificWorker::rgbd_setRegistration ( const QString& server, Registration value )
{
	QMutexLocker locker ( mutex );
}


Registration SpecificWorker::rgbd_getRegistration ( const QString& server )
{
	QMutexLocker locker ( mutex );
	return RoboCompRGBD::DepthInColor;
}


void SpecificWorker::rgbd_getData ( const QString& server, RoboCompRGBD::imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState )
{
	QMutexLocker locker ( mutex );
	
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


void SpecificWorker::rgbd_getImage ( const QString& server, ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState )
{
	QMutexLocker locker ( mutex );
	
	IM2::RGBD* cam = dynamic_cast<IM2::RGBD*>( d->innerModel->getNode(server) );

	QStringList cameraConfig = cam->im_ifconfig.split ( "," );
	if ( cameraConfig.size() >1 ) {
		uint32_t basePort  = QString ( cam->im_ifconfig.split ( "," ) [1] ).toUInt();
		std::map<uint32_t, DifferentialRobotServer>::iterator base;
		base = d->dfr_servers.find( basePort );
		if( base != d->dfr_servers.end() ) {
			base->second.interface->getBaseState( bState );
		}
		
		uint32_t jointPort = QString ( cam->im_ifconfig.split ( "," ) [0] ).toUInt();
		std::map<uint32_t, JointMotorServer>::iterator joint;
		joint = d->jm_servers.find( jointPort );
		if( joint != d->jm_servers.end() ) {
			RoboCompJointMotor::MotorStateMap newMap;
			joint->second.interface->getAllMotorState ( newMap );
			static RoboCompJointMotor::MotorStateMap backMap = newMap;
			hState = newMap;
			backMap = newMap;
		}
	}

	const IM2::CameraParams p = cam->getParams();

	//static QVec rndm = QVec::gaussianSamples(1000001, 1, 0.0001);

	if ( color.size() != ( uint ) p.width*p.height ) {
		color.resize ( p.width*p.height );
	}
	if ( depth.size() != ( uint ) p.width*p.height ) {
		depth.resize ( p.width*p.height );
	}
	if ( points.size() != ( uint ) p.width*p.height ) {
		points.resize (p. width*p.height );
	}

	const uint8_t *rgb = cam->getColor();
	const float *d = cam->getDepth();

	for ( int i=0; i<p.height; ++i ) {
		for ( int j=0; j<p.width; ++j ) {
			const int index  = j + ( i ) *p.width;
			const int indexI = j + ( p.height-1-i ) * p.width;
			color[index].red         = rgb[3*indexI+0];
			color[index].green       = rgb[3*indexI+1];
			color[index].blue        = rgb[3*indexI+2];
			if ( d[indexI] <= 1. ) {
				depth[index] = ( p.znear*p.zfar / ( p.zfar - d[indexI]* ( p.zfar-p.znear ) ) ) /**rndm((td++)%1000001)*/ ;
			} else {
				depth[i] = NAN;
			}
			points[index].x = ( depth[index] * ( ( float ) j- ( p.width/2. ) ) / p.focal );
			points[index].y = depth[index] * ( (p.height/2.) - float(i) ) / p.focal;
			points[index].z = ( depth[index] );
			points[index].w = 1.;
		}
	}
}
