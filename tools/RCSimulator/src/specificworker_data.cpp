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
#include "specificworker_data.h"

// Qt includes
#include <QDropEvent>
#include <QEvent>
#include <QGLWidget>
#include <QLabel>
#include <QMouseEvent>
#include <QMutexLocker>
#include <QTime>
#include <QWidget>

// OSG includes
#include <osg/io_utils>
#include <osg/BoundingBox>
#include <osg/LineWidth>
#include <osg/Matrixd>
#include <osg/PolygonMode>
#include <osg/TriangleFunctor>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osgText/Font>
#include <osgText/Text>
#include <osgUtil/IntersectVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/IntersectionVisitor>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

// #define INNERMODELMANAGERDEBUG



// ------------------------------------------------------------------------------------------------
// Private methods
// ------------------------------------------------------------------------------------------------

// Refills laserData with new values
RoboCompLaser::TLaserData SpecificWorker::Data::LASER_createLaserData ( const IM2::Laser &laser ) {
	// 	QMutexLocker locker (mutex);
	static RoboCompLaser::TLaserData laserData;
	QString id = laser.id;
	int measures = laser.im_measures;
	float iniAngle = -laser.im_angle/2;
	float finAngle = laser.im_angle/2;
	float_t maxRange = laser.im_max;

	laserData.resize ( measures );

	double angle = finAngle;  //variable to iterate angle increments
	//El punto inicial es el origen del láser
	const osg::Vec3 P = IM2::QVecToOSGVec ( innerModel->laserToWorld ( id, 0, 0 ) );
	const float incAngle = ( fabs ( iniAngle ) + fabs ( finAngle ) ) / ( float ) measures;
	osg::Vec3 Q,R;

	for ( int i=0 ; i<measures; i++ ) {
		laserData[i].angle = angle;
		laserData[i].dist = maxRange;//*1000.;
		laserDataCartArray[id]->operator[] ( i ) = IM2::QVecToOSGVec ( innerModel->laserTo ( id, id, angle, maxRange ) );

		//Calculamos el punto destino
		Q = IM2::QVecToOSGVec ( innerModel->laserTo ( "root", id, maxRange, angle ) );
		//Creamos el segmento de interseccion
		osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector = new osgUtil::LineSegmentIntersector ( osgUtil::Intersector::MODEL, P, Q );
		osgUtil::IntersectionVisitor visitor ( intersector.get() );

		/// Pasando el visitor al root
		viewer->getRootGroup()->accept ( visitor );

		if ( intersector->containsIntersections() and id!="laserSecurity" ) {
			osgUtil::LineSegmentIntersector::Intersection result = * ( intersector->getIntersections().begin() );
			R = result.getWorldIntersectPoint(); // in world space

			R.x() = R.x() - P.x();
			R.y() = R.y() - P.y();
			R.z() = R.z() - P.z();
			const float dist = sqrt ( R.x() *R.x() + R.y() *R.y() + R.z() *R.z() );

			if ( dist <= maxRange ) {
				laserData[i].dist = dist;//*1000.;
				laserDataCartArray[id]->operator[] ( i ) = IM2::QVecToOSGVec ( innerModel->laserTo ( id, id, dist, laserData[i].angle ) );
			}
		} else {
			laserDataCartArray[id]->operator[] ( i ) = IM2::QVecToOSGVec ( innerModel->laserTo ( id, id, maxRange, laserData[i].angle ) );
		}
		angle -= incAngle;
	}

	///what does it mean? the point of the laser robot.
	laserDataCartArray[id]->operator[] ( measures ) = IM2::QVecToOSGVec ( innerModel->laserTo ( id, id, 0.0001, 0.001 ) );

	return laserData;
}


///--- useful functions.
IM2::Node* SpecificWorker::Data::getNode ( const QString &id, const QString &msg ) {
	IM2::Node *node = innerModel->getNode ( id );
	if ( node==NULL ) {
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::NonExistingNode;
		std::ostringstream oss;
		oss << msg.toStdString() << " error: Node " << id.toStdString() << " does not exist.";
		err.text = oss.str();
		throw err;

	} else {
		return node;
	}
}


void SpecificWorker::Data::checkOperationInvalidNode ( IM2::Node *node,QString msg ) {
	if ( node==NULL ) {
		#ifdef INNERMODELMANAGERDEBUG
		qDebug() <<msg<<node->id<<"is not transform type";
		#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::OperationInvalidNode;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: Node " << node->id.toStdString() <<" is not of the type require";
		err.text = oss.str();
		throw err;
	}
}


void SpecificWorker::Data::checkNodeAlreadyExists ( const QString &id, const QString &msg ) {
	if ( innerModel->getIDKeys().contains ( id ) ) {
		#ifdef INNERMODELMANAGERDEBUG
			qDebug ( "item already exist. %s\n", id.toStdString().c_str() );
		#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::NodeAlreadyExists;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: Node " << id.toStdString() << " already exists.";
		err.text = oss.str();
		throw err;
	}
}


void SpecificWorker::Data::checkInvalidMeshValues ( RoboCompInnerModelManager::meshType m, QString msg ) {
	///check Scale
	osg::Node *osgMesh = osgDB::readNodeFile ( m.meshPath );
	if ( m.scaleX<0.0 or m.scaleY <0.0 or m.scaleZ <0.0 ) {
#ifdef INNERMODELMANAGERDEBUG
		qDebug() <<"--- Fatal:"<<msg<<"Scale can not be negative";
		qDebug() <<"m.scaleX "<<m.scaleX<<"m.scaleY"<<m.scaleY<<"m.scaleZ"<<m.scaleZ;
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::InvalidValues;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: Scale (" << m.scaleX << ", " << m.scaleY << ", " << m.scaleZ << ") is invalid.";
		err.text = oss.str();
		throw err;
	}
	///check valid osg Node.
	else if ( osgMesh==NULL ) {
#ifdef INNERMODELMANAGERDEBUG
		qDebug() <<"--- Fatal:"<<msg<<"meshPath:"<<QString::fromStdString ( m.meshPath ) <<"does not exist or no it is a type valid for his OpenSceneGraph.";
#endif

		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::InvalidPath;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: meshPath: " << m.meshPath << ", " <<"does not exist or no it is a type valid for his OpenSceneGraph.";
		err.text = oss.str();
		throw err;
	}
}


void SpecificWorker::Data::AttributeAlreadyExists ( IM2::Node* node, QString attributeName, QString msg ) {
	if ( node->im_attributes.contains ( attributeName ) ) {
#ifdef INNERMODELMANAGERDEBUG
		qDebug ( "attribute already exist. %s\n", attributeName.toStdString().c_str() );
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::AttributeAlreadyExists;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: attribute " << attributeName.toStdString() << " already exists." <<" in node "<<node->id.toStdString();
		err.text = oss.str();
		throw err;
	}
}


void SpecificWorker::Data::NonExistingAttribute ( IM2::Node* node, QString attributeName, QString msg ) {
	if ( node->im_attributes.contains ( attributeName ) ==false ) {
#ifdef INNERMODELMANAGERDEBUG
		qDebug ( "attribute NO exist. %s\n", attributeName.toStdString().c_str() );
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::AttributeAlreadyExists;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error: attribute " << attributeName.toStdString() << " NO exists."<<" in node "<<node->id.toStdString();
		err.text = oss.str();
		throw err;
	}
}


void SpecificWorker::Data::getRecursiveNodeInformation ( RoboCompInnerModelManager::NodeInformationSequence& nodesInfo, IM2::Node* node ) {
	/// Add current node information
	RoboCompInnerModelManager::NodeInformation ni;
	ni.id = node->id.toStdString();

	if ( node->im_parent ) {
		ni.parentId = node->im_parent->id.toStdString();
	} else {
		ni.parentId = "";
	}
	ni.nType = getNodeType ( node );

	RoboCompInnerModelManager::AttributeType a;
	foreach ( const QString &str, node->im_attributes.keys() ) {
		a.type=node->im_attributes.value ( str ).type.toStdString();
		a.value=node->im_attributes.value ( str ).value.toStdString();
		ni.attributes[str.toStdString()]=a;
	}
	nodesInfo.push_back ( ni );

	/// Recursive call for all children
	QSet<IM2::Node *>::iterator child;
	for ( child = node->im_children.begin(); child != node->im_children.end(); child++ ) {
		getRecursiveNodeInformation ( nodesInfo, *child );
	}
}


RoboCompInnerModelManager::NodeType SpecificWorker::Data::getNodeType ( IM2::Node* node ) {
	if ( dynamic_cast<IM2::HingeJoint*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::Joint;
	} else if ( dynamic_cast<IM2::DifferentialRobot*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::DifferentialRobot;
	} else if ( dynamic_cast<IM2::Plane*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::Plane;
	} else if ( dynamic_cast<IM2::RGBD*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::RGBD;
	} else if ( dynamic_cast<IM2::Camera*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::Camera;
	} else if ( dynamic_cast<IM2::IMU*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::IMU;
	} else if ( dynamic_cast<IM2::Laser*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::Laser;
	} else if ( dynamic_cast<IM2::PointCloud*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::PointCloud;
	} else if ( dynamic_cast<IM2::GenericJoint*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::Transform;
	#ifndef USE_BULLET
	} else if ( dynamic_cast<IM2::Mesh*> ( node ) != NULL ) {
		return RoboCompInnerModelManager::Mesh;
	#endif
	} else {
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::InternalError;
		std::ostringstream oss;
		oss << "RoboCompInnerModelManager::getNodeType() error: Type of node " << node->id.toStdString() << " is unknown.";
		err.text = oss.str();
		throw err;
	}
	//return RoboCompInnerModelManager::Node;
}


// Update all the joint positions
void SpecificWorker::Data::updateJoints( const float delta ) {
	QHash<QString, JointMovement>::const_iterator iter;
	for ( iter = jointMovements.constBegin() ; iter != jointMovements.constEnd() ; ++iter ) {
		IM2::HingeJoint* joint = this->innerModel->getHingeJoint ( iter.key() );
		const float angle = joint->getAngle();
		const float amount = fminf ( fabsf ( iter->endPos - angle ), iter->endSpeed * delta);
		switch ( iter->mode ) {
		case JointMovement::FixedPosition:
			joint->setAngle( iter->endPos );
			break;
		case JointMovement::TargetPosition:
			if( iter->endPos > angle )
				joint->setAngle( angle + amount );
			else if( iter->endPos < angle )
				joint->setAngle( angle - amount );
			break;
		case JointMovement::TargetSpeed:
			joint->setAngle ( angle + iter->endSpeed * delta );
			break;
		default:
			break;
		}
	}
}


void SpecificWorker::Data::addDFR ( IM2::DifferentialRobot* node ) {
	const uint32_t port = node->im_port;
	if( dfr_servers.count( port ) == 0 ) {
		dfr_servers.insert( std::pair<uint32_t, DifferentialRobotServer>( port, DifferentialRobotServer( communicator, worker, port ) ) );
	}
	dfr_servers.at(port).add( node );
}


void SpecificWorker::Data::addIMU ( IM2::IMU* node ) {
	const uint32_t port = node->im_port;
	if( imu_servers.count( port ) == 0 ) {
		imu_servers.insert( std::pair<uint32_t, IMUServer>( port, IMUServer( communicator, worker, port ) ) );
	}
	imu_servers.at(port).add( node );
}


void SpecificWorker::Data::addJM( IM2::HingeJoint* node ) {
	const uint32_t port = node->im_port;
	if( jm_servers.count( port ) == 0 ) {
		jm_servers.insert( std::pair<uint32_t, JointMotorServer>( port, JointMotorServer( communicator, worker, port ) ) );
	}
	jm_servers.at(port).add( node );
}


void SpecificWorker::Data::addLaser ( IM2::Laser* node ) {
	const uint32_t port = node->im_port;
	if( laser_servers.count( port ) == 0 ) {
		laser_servers.insert( std::pair<uint32_t, LaserServer>( port, LaserServer( communicator, worker, port ) ) );
	}
	laser_servers.at(port).add( node );
}


void SpecificWorker::Data::addRGBD ( IM2::RGBD* node ) {
	const uint32_t port = node->im_port;
	if( rgbd_servers.count( port ) == 0 ) {
		rgbd_servers.insert( std::pair<uint32_t, RGBDServer>( port, RGBDServer( communicator, worker, port ) ) );
	}
	rgbd_servers.at(port).add( node );
}


void SpecificWorker::Data::removeJM( IM2::HingeJoint* node ) {
	std::map<uint32_t, JointMotorServer>::iterator it;
	for ( it = jm_servers.begin(); it != jm_servers.end(); ++it ) {
		it->second.remove( node );
		// TODO: arreglar
// 		if ( it->second.empty() ) {
// 			worker->scheduleShutdown( &(it->second) );
// 			servers.erase( it );
// 		}
	}
}


void SpecificWorker::Data::walkTree( IM2::Node* node ) {
	if ( node == NULL )
	node = innerModel->getRoot();

	QSet<IM2::Node*>::iterator it;
	for ( it = node->im_children.begin() ; it != node->im_children.end() ; ++it ) {
		IM2::HingeJoint* jointNode = dynamic_cast<IM2::HingeJoint *>(*it);
		if ( jointNode != NULL ) {
			qDebug() << "Joint " << jointNode->id;
			addJM(jointNode);
		}
		
		IM2::DifferentialRobot* differentialNode = dynamic_cast<IM2::DifferentialRobot *>(*it);
		if ( differentialNode != NULL ) {
			qDebug() << "DifferentialRobot " << differentialNode->id << differentialNode->im_port;
			addDFR(differentialNode);
		}
		
		IM2::IMU* imuNode = dynamic_cast<IM2::IMU *>(*it);
		if ( imuNode != NULL ) {
			qDebug() << "IMU " << imuNode->id << imuNode->im_port;
			addIMU(imuNode);
		}
		
		IM2::Laser* laserNode = dynamic_cast<IM2::Laser *>(*it);
		if ( laserNode != NULL ) {
			qDebug() << "Laser " << laserNode->id << laserNode->im_port;
			addLaser(laserNode);
		}
		
		IM2::RGBD* rgbdNode = dynamic_cast<IM2::RGBD *>(*it);
		if ( rgbdNode != NULL ) {
			qDebug() << "RGBD " << rgbdNode->id << rgbdNode->im_port;
			addRGBD(rgbdNode);
		}
		walkTree(*it);
	}
}


void SpecificWorker::Data::scheduleShutdown( JointMotorServer *j )
{
	jointServersToShutDown.push_back(j);
}
