/****************************************************************************
#                                                                           #
#                    Copyright (C) 2008                                     #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/
#include "robotinnermodel.h"

/**
 * \brief Default constructor
 */
RobotInnerModel::RobotInnerModel()
{
//	leftCamera(), leftEx(), rightEx(),  hStateAnt(), base(), bState(), bStateAnt(),  hState(),  bPose(), bPoseVar(), STATE_DIM(3)
}

/**
 * \brief Copy constructor
 * @param rob
 */
RobotInnerModel::RobotInnerModel(const RobotInnerModel & rob)
{
	qFatal("The class RobotInnerModel is outdated.");
	leftCamera = rob.leftCamera;
	thirdCamera = rob.thirdCamera;
	rightCamera = rob.rightCamera;
	robotToCentral = rob.robotToCentral;
	centralToLeftMotor = rob.centralToLeftMotor;
	centralToThirdMotor = rob.centralToThirdMotor;
	centralToRightMotor = rob.centralToRightMotor;
	leftMotorToLeftCamera = rob.leftMotorToLeftCamera;
	rightMotorToRightCamera = rob.rightMotorToRightCamera;
	base = rob.base;
	bState = rob.bState;
	hState = rob.hState;
	bStateAnt = rob.bStateAnt;
	hStateAnt = rob.hStateAnt;
	robotRadius = rob.robotRadius;
	laser = rob.laser;
	fundamental = rob.fundamental;
	essential = rob.essential;
	geometricCenter = rob.geometricCenter;
}

/**
 * \brief Constructor used to initialize a specific named robot.
 * In future versions al kinematic data and dimensions will be read from a XML file
 */
RobotInnerModel::RobotInnerModel(QString name)
{
	init(name);
}

/**
 * \brief Destructor
 */
RobotInnerModel::~RobotInnerModel()
{
}

/**
  @brief Initialization of robot geometry. Should read a xml file form disk

  @param rob Name of robot tobe instantiated
  @param mesDim_ deprecated
  @see updatePropioception
 */
void RobotInnerModel::init(const QString & rob )
{
	qDebug() << "RobotInnerModel::init() -> Entering init";

	if( rob == "Pulguita1")
	{
		robotRadius = 200.;
		geometricCenter = QVec::vec3(0., 0., -120.); // Position of geometric center wrt origin

		MovingRobot MR(0.); // Moving robot at 0,0,
		MR.print("MR");
		///BASE
		base.init();
		base.print("base");
		robotRadius = 200.; // Standard radius for Robex
		//init bState
		bState.x = bState.z = bState.alpha = 0.;
		bStateAnt = bState;
		///HEAD
		leftCamera.set( 300, 300, 160., 120. );
		leftCamera.print("LeftCamera");
		rightCamera.set( 300, 300, 160., 120. );
		rightCamera.print("RightCamera");
		thirdCamera.set( 400, 400, 160., 120. );
		robotToCentral.set( 0., 0., 0., 375., 100.);
		robotToCentral.print("robotToCentral");
		centralToLeftMotor.set( 0., 0., -72.5, 0., 0.);
		centralToLeftMotor.print("centralToLeftMotor"); // Common Tilt
		centralToRightMotor.set( 0., 0., 72.5, 0., 0.);
		centralToRightMotor.print("centralToRightMotor"); // Common Tilt
		centralToThirdMotor.set( 0., 0., 0., -353.5, -30.);
		leftMotorToLeftCamera.set( 0., 0., 0., 0., 0.);
		rightMotorToRightCamera.set( 0., 0., 0., 0., 0.);
		thirdMotorToThirdCamera.set( 0., 0., 0., 0., 0.);
		///STEREO GEOMETRY
		essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
		essential.print("Essential");
		fundamental.set( essential, leftCamera, rightCamera);
		fundamental.print("Fundamental");
		/// Laser
		QMat T(3);
		T(0) = 0.;
		T(1) = 140.;
		T(2) = 60.;
		laser.init( 0., 0., T);
	}
	else if ( rob == "Tallu" )
	{
		robotRadius = 200.;
		geometricCenter = QVec::vec3(0., 0., -120.);
		MovingRobot MR (0.); // Moving robot at 0,0,
		///BASE
		base.init();
		robotRadius = 200.; // Standard radius for Robex
		bState.x = bState.z = bState.alpha = 0.;
		bStateAnt = bState;
		///HEAD
		leftCamera.set(400, 400, 120., 160.);
		rightCamera.set(400, 400, 120., 160.);
		robotToCentral.set(0., 0., 0., 180., 0.);
		centralToLeftMotor.set(0., 0., 0., 120., 0.);
		centralToRightMotor.set(0., 0., 0., -120., 0.);
		leftMotorToLeftCamera.set( 0., 0., 0., 0., 0.);
		rightMotorToRightCamera.set( 0., 0., 0., 0., 0.);
		thirdMotorToThirdCamera.set( 0., 0., 0., 0., 0.);
		///STEREO GEOMETRY
		essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
		fundamental.set( essential, leftCamera, rightCamera);
		/// Laser
		QMat T(3);
		T(0) = 0.;
		T(1) = 140.;
		T(2) = 60.;
		laser.init( 0., 0., T);
	}

	else if( rob == "Speedy1")
	{
		///BASE
		base.init();
		base.print("base");
		robotRadius = 200.; // Standard radius for Robex
		geometricCenter = QVec::vec3(0., 0., -120.);	// Position of geometric center wrt origin

		//init bState
		bState.x = 0.; bState.z = 0.; bState.alpha= 0.;
		bStateAnt = bState;

		///HEAD

		leftCamera.set( 265, 265, 160., 120. );
		leftCamera.print("LeftCamera");
		rightCamera.set( 265, 265, 160., 120. );
		rightCamera.print("RightCamera");

		robotToCentral.set( 0., 0., 0., 333., 50.);
		robotToCentral.print("robotToCentral");
		centralToLeftMotor.set( 0., 0., -77.5, 0., 0.);
		centralToLeftMotor.print("centralToLeftMotor"); // Common Tilt
		centralToRightMotor.set( 0., 0., 77.5, 0., 0.);
		centralToRightMotor.print("centralToRightMotor"); // Common Tilt

		leftMotorToLeftCamera.set( 0., 0., 0., 0., 0.);
		leftMotorToLeftCamera.print("leftMotorToLeftCamera"); // Left Pan
		rightMotorToRightCamera.set( 0., 0., 0., 0., 0.);
		rightMotorToRightCamera.print("rightMotorToRightCamera"); // Right Pan

		///STEREO GEOMETRY
		essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
		essential.print("Essential");
		fundamental.set( essential, leftCamera, rightCamera);
		fundamental.print("Fundamental");

		/// Laser
		QMat T(3);
		T(0) = 0.;
		T(1) = 140.;
		T(2) = 60.;
		laser.init( 0., 0., T);

	}
	else if( rob == "Ramon")
	{
		base.init();
		base.print("base");
		robotRadius = 200.; // Standard radius for Robex
		geometricCenter = QVec::vec3(0., 0., -120.); // Position of geometric center wrt origin

		//init bState
		bState.x = 0.; bState.z = 0.; bState.alpha= 0.;
		bStateAnt = bState;

		///HEAD
		int FocusLeft, FocusRight;
		FocusLeft = FocusRight = 276;
		leftCamera.set( FocusLeft, FocusLeft, 160., 120. );
		leftCamera.print("LeftCamera");
		rightCamera.set( FocusRight, FocusRight, 160., 120. );
		rightCamera.print("RightCamera");

		///+BODY
		robotToCentral.set( 0., 0., 0., 425., 75.);
		robotToCentral.print("robotToCentral");
		centralToLeftMotor.set( 0., 0., -80., 0., 0.);
		centralToLeftMotor.print("centralToLeftMotor"); //Common Tilt
		centralToRightMotor.set( 0., 0., 80., 0., 0.);
		centralToRightMotor.print("centralToRightMotor"); //Common Tilt

		leftMotorToLeftCamera.set( 0., 0., 0., 0., -10.);
		leftMotorToLeftCamera.print("leftMotorToLeftCamera"); // Left Pan
		rightMotorToRightCamera.set( 0., 0., 0., 0., -10.);
		rightMotorToRightCamera.print("rightMotorToRightCamera"); // Right Pan

		MovingRobot MR(0.); //Moving robot at 0,0,0
	}

else if( rob == "SpeedyBeta")
	{
		base.init();
		base.print("base");
		robotRadius = 200.; // Standard radius for Robex
		geometricCenter = QVec::vec3(0., 0., -120.); // Position of geometric center wrt origin

		//init bState
		bState.x = 0.; bState.z = 0.; bState.alpha= 0.;
		bStateAnt = bState;

		///HEAD
		int FocusLeft, FocusRight;
		FocusLeft = FocusRight = 426;
		leftCamera.set( FocusLeft, FocusLeft, 160., 120. );
		leftCamera.print("LeftCamera");
		rightCamera.set( FocusRight, FocusRight, 160., 120. );
		rightCamera.print("RightCamera");

		///+BODYhttp://www.iheartrobotics.com/
		robotToCentral.set( 0., 0., 0., 413., 57.5 );
		robotToCentral.print("robotToCentral");
		centralToLeftMotor.set( 0., 0., -77.5, 0., 0.);
		centralToLeftMotor.print("centralToLeftMotor"); //Common Tilt
		centralToRightMotor.set( 0., 0., 77.5, 0., 0.);
		centralToRightMotor.print("centralToRightMotor"); //Common Tilt

		leftMotorToLeftCamera.set( 0., 0., 0., 0., 0.);
		leftMotorToLeftCamera.print("leftMotorToLeftCamera"); // Left Pan
		rightMotorToRightCamera.set( 0., 0., 0., 0., 0.);
		rightMotorToRightCamera.print("rightMotorToRightCamera"); // Right Pan

		MovingRobot MR(0.); //Moving robot at 0,0,0
	}


	else
		qFatal("Innermodel: unknow robot");

	qDebug() << "RobotInnerModel::init() -> Init completed";

}

/**
 * \brief Updates innerModel from sensorial data coming from BaseComp, CamMotionComp and LaserComp
 * @param bState_ kinematic state of robot moving base
 * @param hState_ kinematic state of stereo head
 * @param lData current data read from laser
 */
void RobotInnerModel::updatePropioception(const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompCamMotion::THeadState & hState_, const RoboCompLaser::TLaserData & lData)
{
	bStateAnt = bState;
	hStateAnt = hState;

	bState = bState_;
	hState = hState_;

	laserData = lData;
	//innerLaserData = lData;

 	base.setRT( bState.x, bState.z, bState.alpha);
 	centralToLeftMotor.setR( hState.leftTilt.pos , 0.);
 	centralToRightMotor.setR( hState.rightTilt.pos, 0.);
	leftMotorToLeftCamera.setR( 0., hState.left.pos );
	rightMotorToRightCamera.setR( 0., hState.right.pos );

	essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
 	fundamental.set( essential, leftCamera, rightCamera);

 }
void RobotInnerModel::updatePropioception(const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompCommonHead::THeadState & hState_, const RoboCompLaser::TLaserData & lData)
{
	bStateAnt = bState;
	hStateAnt = hState;

	bState = bState_;

	//pasar d THeadState nuevo al antiguo
	hState = headnuevoToantiguo(hState_);

	laserData = lData;
	//innerLaserData = lData;

 	base.setRT( bState.x, bState.z, bState.alpha);
 	centralToLeftMotor.setR( hState.leftTilt.pos , 0.);
 	centralToRightMotor.setR( hState.rightTilt.pos, 0.);
	leftMotorToLeftCamera.setR( 0., hState.left.pos );
	rightMotorToRightCamera.setR( 0., hState.right.pos );

	essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
 	fundamental.set( essential, leftCamera, rightCamera);

 }
/**
 * \brief Updates innerModel with sensorial state data coming from BaseComp, CamMotionComp
 * @param bState_ kinematic state of robot moving base
 * @param hState_ kinematic state of stereo head
 */
void RobotInnerModel::updatePropioception(const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompCamMotion::THeadState & hState_)
{
	bStateAnt = bState;
	hStateAnt = hState;

	bState = bState_;
	hState = hState_;

 	base.setRT( bState.x, bState.z, bState.alpha);
 	centralToLeftMotor.setR( hState.tilt.pos , 0.);
 	centralToRightMotor.setR( hState.tilt.pos, 0.);
	leftMotorToLeftCamera.setR( 0., hState.left.pos );
	rightMotorToRightCamera.setR( 0., hState.right.pos );

 	essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
 	fundamental.set( essential, leftCamera, rightCamera);
}

void RobotInnerModel::updatePropioception(const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompCommonHead::THeadState & hState_)
{

	bStateAnt = bState;
	hStateAnt = hState;

	bState = bState_;

	//pasar d THeadState nuevo al antiguo
	hState = headnuevoToantiguo(hState_);


 	base.setRT( bState.x, bState.z, bState.alpha);
 	centralToLeftMotor.setR( hState.leftTilt.pos , 0.);
 	centralToRightMotor.setR( hState.rightTilt.pos, 0.);
	leftMotorToLeftCamera.setR( 0., hState.left.pos );
	rightMotorToRightCamera.setR( 0., hState.right.pos );

 	essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
 	fundamental.set( essential, leftCamera, rightCamera);
 }


void RobotInnerModel::updatePropioception(const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompJointMotor::MotorStateMap & hState_)
{

	bStateAnt = bState;
	hStateAnt = hState;

	bState = bState_;

	//pasar d THeadState nuevo al antiguo
	hState = headnuevoToantiguo(hState_);


 	base.setRT( bState.x, bState.z, bState.alpha);
 	centralToLeftMotor.setR( hState.leftTilt.pos , 0.);
 	centralToRightMotor.setR( hState.rightTilt.pos, 0.);
	leftMotorToLeftCamera.setR( 0., hState.left.pos );
	rightMotorToRightCamera.setR( 0., hState.right.pos );

 	essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
 	fundamental.set( essential, leftCamera, rightCamera);
 }


/**
 * \brief Updates innerModel with Base state data coming from BaseComp
 * @param bState_ kinematic state of robot moving base
 */
void RobotInnerModel::updatePropioception(const RoboCompDifferentialRobot::TBaseState & bState_)
{
	bStateAnt = bState;
	bState = bState_;

 	base.setRT( bState.x, bState.z, bState.alpha);
}

/**
 * \brief Updates innerModel with float data data. Overloaded method
 * @param bState_ kinematic state of robot moving base
 */
void RobotInnerModel::updatePropioception(const float x, const float z, const float alpha)
{

	bStateAnt = bState;
	bState.x = x;
	bState.z = z;
	bState.alpha = alpha;

 	base.setRT( x, z, alpha);
}

/**
 * \brief Updates innerModel with head state data coming from CamMotion
 * @param hState_ kinematic state of stereo head
 */
void RobotInnerModel::updatePropioception(const RoboCompCamMotion::THeadState & hState_)
{
	hStateAnt = hState;
	hState = hState_;

	centralToLeftMotor.setR( hState.tilt.pos , 0.);
	centralToThirdMotor.setR( 0, 0.);
 	centralToRightMotor.setR( hState.tilt.pos, 0.);
	leftMotorToLeftCamera.setR( 0., hState.left.pos );
	thirdMotorToThirdCamera.setR(0., 0 );
	rightMotorToRightCamera.setR( 0., hState.right.pos );

 	essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
 	fundamental.set( essential, leftCamera, rightCamera);

 }

/**
 * \brief Updates innerModel with head state data coming from CamMotion
 * @param hState_ kinematic state of stereo head
 */
void RobotInnerModel::updatePropioception(const RoboCompCommonHead::THeadState & hState_)
{
	hStateAnt = hState;

	//pasar d THeadState nuevo al antiguo
	hState = headnuevoToantiguo(hState_);

	centralToLeftMotor.setR( hState.leftTilt.pos , 0.);
	centralToThirdMotor.setR( 0, 0.);
 	centralToRightMotor.setR( hState.rightTilt.pos, 0.);
	leftMotorToLeftCamera.setR( 0., hState.left.pos );
	thirdMotorToThirdCamera.setR(0., 0 );
	rightMotorToRightCamera.setR( 0., hState.right.pos );

 	essential.set(rightCamRotationToLeftCam(), rightCamTranslationToLeftCam());
 	fundamental.set( essential, leftCamera, rightCamera);

 }


/**
 * \brief Updates innerModel from incremental displacement and rotation coming from BaseComp
 * This method does the same as BaseComp for updating base odometry. It is supplied to be used in cloned copies of InnerModel that follow virtual trayectories
 * @param incAng increment in angular position from last measurement
 * @param incAdv increment in forward translation from last measurement
 */
void RobotInnerModel::updateIncrementalPropioception(float incAng, float incAdv)
{
		base.updateRT(incAng,incAdv);
		QMat tr(3);
		tr = base.getTr();
		bState.x = tr(0);
		bState.z = tr(2);
		bState.alpha = base.getAlpha();
		bState.adv = sqrt(pow(bState.x - bStateAnt.x,2) + pow(bState.z-bState.z,2));
		bState.rot = bState.alpha - bStateAnt.alpha;
		bStateAnt = bState;
}

/// LASER RELATED

/**
 * Local laser measure (angle,range) to World RF coordinate conversión
 * @param p 2D vector with bearing (rads) and range (mm)
 * @return 3D point en World RS
 */
QVec RobotInnerModel::laserToWorld(const QVec & p) const
{
	return base.robotToWorld( laser.cameraToBase( p ) );
}

/**
 * \brief Local laser measure with index i in laser array is converted to World RS
 * @param i indexof laser array
 * @return 3Dpoint in World RS
 */
QVec RobotInnerModel::laserToWorld(int i) const
{

	return laserToWorld( laserData[i].dist, laserData[i].angle );
}

/**
 * \brief Local laser measure of range r and angle alpha is converted to World RS
 * @param r range measure
 * @param alpha angle measure
 * @return 3-vector of x,y,z coordinates un WRS
 */
QVec RobotInnerModel::laserToWorld( float r, float alpha) const
{
	QVec p(3);

	p(0) =  r * sin ( alpha ) ;
	p(2) =  r * cos ( alpha ) ;
	p(1) =  0 ; // Laser reference system
	return base.robotToWorld( laser.inverse( p ) );
}

/**
 * \brief Converts a 3D point en WRS to Laser reference system
 * @param world 3D coordinates of a world point as a 3-vector
 * @return 3D coordinates of given point seen from Laser reference system
 */
QVec RobotInnerModel::worldToLaser(const QVec & world) const
{
	QVec r(2), q(3);
	q = laser.direct( base.direct(world) );
	//to polar coors
	r(1) = atan2(q(0),q(2));
	r(0) = q.norm2();
	return r;
}

/**
 * \brief Converts a 3D point in Laser (range,angle) coordinates to Robot reference system
 * @param r range measure
 * @param alpha angle measure
 * @return 3-vector of 3D point in Robot reference system
 */
QVec RobotInnerModel::laserToBase(float r, float alpha) const
{
	QVec p(3);
	p(0) =   r * sin ( alpha ) ;
	p(2) =   r * cos ( alpha ) ;
	p(1) =  0;
	return laser.cameraToBase( p ) ;
}

/**
 * \brief Converts a laser measure given by its index in laser array to Robot reference system
 * @param i index in laser array as defined in TLaserData::TLaserData
 * @return 3-vector of 3D point in Robot reference system
 */
QVec RobotInnerModel::laserToBase(int i) const
{
	QVec p(3);
	p(0) =  laserData[i].dist * sin ( laserData[i].angle ) ;
	p(2) =  laserData[i].dist * cos ( laserData[i].angle ) ;
	return laser.cameraToBase( p ) ;
}

///Getters for local state

/**
 * \brief Returns current copy of Base odometry as a 3-vector: x,z,alpha
 * @return 3-vector with base state as x,z,alpha
 */
QVec RobotInnerModel::getBaseOdometry() const
{
	QVec odom(3);
	odom(0) = bState.x;
	odom(1) = bState.z;
	odom(2) = bState.alpha;
	return odom;
}

/**
 * \brief Returns former (in t-1) copy of Base odometry as a 3-vector: x,z,alpha
 * @return 3-vector with base state as x,z,alpha
 */
QVec RobotInnerModel::getBaseOdometryAnt() const
{
	QVec odom(3);
	odom(0) = bStateAnt.x;
	odom(1) = bStateAnt.z;
	odom(2) = bStateAnt.alpha;
	return odom;
}

/// CAMERAS RELATED

/**
 * \brief Projects a 3D world point in World reference system to left camera pixel coordinates using left camera projection matrix and current head extrinsics
 * @param p 3D world coordinate.
 * @return 3-vector with 2D image coordinate and depth to original 3D point measured from left camera reference system
 */
QVec RobotInnerModel::projectFromWorldToLeftCam(const QVec & p) const
{
 	QVec pe = leftMotorToLeftCamera.direct( centralToLeftMotor.direct( robotToCentral.direct( base.worldToRobot( p ) ) ) );
	QVec pc = leftCamera.project( pe );
	QVec res(3);
	res(0) = pc(0);
	res(1) = pc(1);
	res(2) = pe.norm2();  // Distance to point goes in third position
	return res;

}

/**
 * \brief Projects a 3D point in Robot reference system to left camera pixel coordinates using left camera projection matrix and current head extrinsics
 * @param p 3D point in Robot reference system
 * @return 3-vector with 2D image coordinates and depth to original 3D point measured from left camera reference system
*/
QVec RobotInnerModel::projectFromRobotToLeftCam(const QVec & p) const
{
	QVec pe = leftMotorToLeftCamera.direct( centralToLeftMotor.direct( robotToCentral.direct( p ) ) );
	QVec pc = leftCamera.project( pe );
	QVec res(3);
	res(0) = pc(0);
	res(1) = pc(1);
	res(2) = pe.norm2(); // Distance to point goes in third position
	return res;
}

/**
 * \brief Projects a 3D world point in World reference system to third camera pixel coordinates using third camera projection matrix and current head extrinsics
 * @param p 3D world coordinate.
 * @return 3-vector with 2D image coordinate and depth to original 3D point measured from third camera reference system
 */
QVec RobotInnerModel::projectFromWorldToThirdCam(const QVec & p) const
{
 	QVec pe = thirdMotorToThirdCamera.direct( centralToThirdMotor.direct( robotToCentral.direct( base.worldToRobot( p ) ) ) );
	QVec pc = thirdCamera.project( pe );
	QVec res(3);
	res(0) = pc(0);
	res(1) = pc(1);
	res(2) = pe.norm2();  // Distance to point goes in third position
	return res;

}

/**
 * \brief Projects a 3D point in Robot reference system to third camera pixel coordinates using third camera projection matrix and current head extrinsics
 * @param p 3D point in Robot reference system
 * @return 3-vector with 2D image coordinates and depth to original 3D point measured from third camera reference system
*/
QVec RobotInnerModel::projectFromRobotToThirdCam(const QVec & p) const
{
	QVec pe = thirdMotorToThirdCamera.direct( centralToThirdMotor.direct( robotToCentral.direct( p ) ) );
	QVec pc = thirdCamera.project( pe );
	QVec res(3);
	res(0) = pc(0);
	res(1) = pc(1);
	res(2) = pe.norm2(); // Distance to point goes in third position
	return res;
}

/**
 * \brief Projects a 3D world point in World reference system to right camera pixel coordinates using right camera projection matrix and current head extrinsics
 * @param p 3D world coordinate.
 * @return 3-vector of 2D image coordinates and depth to original 3D point measured from right camera reference system
 */
QVec RobotInnerModel::projectFromWorldToRightCam(const QVec & p) const
{
	QVec pe = rightMotorToRightCamera.direct( centralToRightMotor.direct( robotToCentral.direct( base.worldToRobot( p ) ) ) );
	QVec pc = rightCamera.project( pe );
	QVec res(3);
	res(0) = pc(0);
	res(1) = pc(1);
	res(2) = pe.norm2(); // Distance to point goes in third position
	return res;
}

/**
 * \brief Projects a 3D point in Robot reference system to right camera pixel coordinates using right camera projection matrix and current head extrinsics
 * @param p 3D point in Robot reference system
 * @return 3-vector with 2D image coordinates and depth to original 3D point measured from right camera reference system
*/
QVec RobotInnerModel::projectFromRobotToRightCam(const QVec & p) const
{
	QVec pe = rightMotorToRightCamera.direct( centralToRightMotor.direct( robotToCentral.direct( p ) ) );
	QVec pc = rightCamera.project( pe );
	QVec res(3);
	res(0) = pc(0);
	res(1) = pc(1);
	res(2) = pe.norm2(); // Distance to point goes in third position
	return res;
}

/**
 * \brief Computes de 3D coordinates in World RS of a 2D image point in right camera assumed to belong to the floor
 * @param p 2D image point of a 3D point on the floor seen from the right camera
 * @return 3-vector of 3D world point coordinates
 */
QVec RobotInnerModel::rightCamFromFloorToWorld(const QVec & p) const
{
	return robotToWorld( rightCamFromFloorToRobot( p ) );
}

 /**
 * \brief Computes the 3D coordinates in World RS of a 2D image point in left camera assumed to belong to the floor
 * @param p 2D image point of a 3D point on the floor seen from the left camera
 * @return 3-vector with 3D world point coordinates
 */
QVec RobotInnerModel::leftCamFromFloorToWorld(const QVec & p) const
{
	return robotToWorld( leftCamFromFloorToRobot( p ) );
}

 /**
 * \brief Computes the 3D coordinates in World RS of a 2D image point in third camera assumed to belong to the floor
 * @param p 2D image point of a 3D point on the floor seen from the third camera
 * @return 3-vector with 3D world point coordinates
 */
QVec RobotInnerModel::thirdCamFromFloorToWorld(const QVec & p) const
{
	return robotToWorld( thirdCamFromFloorToRobot( p ) );
}

/**
 * \brief Computes the 3D coordinates in Robot reference system of a 2D image point in left camera assumed to belong to the floor
 * @param p 2D image point of a 3D point on the floor seen from the left camera
 * @return 3-vector of 3D robot point coordinates
 */
QVec RobotInnerModel::leftCamFromFloorToRobot(const QVec & p) const
{
	// En el sistema de referencia de la camara
	// (X,Y,Z) = a*(x,y,1); donde Y = h

	//get angles in camara RS
	QVec angles = leftCamera.getAnglesHomogeneous( p );
	//leftMotorToLeftCamera.getR() * centralToLeftMotor.getR() ;
	//get angles in robot RS
	//float tilt = tan(angles(1) +  centralToLeftMotor.getRxValue());
//	float leftPan = tan(angles(0)) ;//+ leftMotorToLeftCamera.getRyValue());

	QVec rr(2);
	rr(0) =  p(0) - 160;
	rr(1) =  -p(1) + 120;

	float tilt = (rr(1) / leftCamera.getFocal()) +  centralToLeftMotor.getRxValue();
	float leftPan = rr(0) / leftCamera.getFocal()+ leftMotorToLeftCamera.getRyValue();

	/** TODO: The following line is not enough, getHorizonLine should be used to check this restriction. */
	//Q_ASSERT( tilt < 0);  // we want something below the horizon line

	float h = leftCamToRobot(QVec::zeros(3))(1); // Height
	float a = -h / tilt;
	QVec r(3);
	r(0) = a * leftPan;
	r(1) = -h;
	r(2) = a;

	return robotToCentral.inverseTr( centralToLeftMotor.inverseTr(leftMotorToLeftCamera.inverseTr(r)));
}

/**
 * \brief Computes the 3D coordinates in Robot reference system of a 2D image point in third camera assumed to belong to the floor
 * @param p 2D image point of a 3D point on the floor seen from the third camera
 * @return 3-vector of 3D robot point coordinates
 */
QVec RobotInnerModel::thirdCamFromFloorToRobot(const QVec & p) const
{
	//get angles in camara RS
	QVec angles = thirdCamera.getAnglesHomogeneous( p );

	QVec rr(2);
	rr(0) =  p(0) - 160;
	rr(1) = -p(1) + 120;

	float tilt = (rr(1) / thirdCamera.getFocal()) +  centralToThirdMotor.getRxValue();
	float thirdPan = rr(0) / thirdCamera.getFocal() + thirdMotorToThirdCamera.getRyValue();

	/** TODO: The following line is not enough, getHorizonLine should be used to check this restriction. */
	//Q_ASSERT( tilt < 0);  // we want something below the horizon line

	float h = thirdCamToRobot(QVec::zeros(3))(1); // Height
	float a = -h / tilt;
	QVec r(3);
	r(0) = a * thirdPan;
	r(1) = -h;
	r(2) = a;

	return robotToCentral.inverseTr( centralToThirdMotor.inverseTr(thirdMotorToThirdCamera.inverseTr(r)));
}

 /**
 * \brief Computes the 3D coordinates in Robot reference system of a 2D image point in roght camera assumed to belong to the floor
 * @param p 2D image point of a 3D point on the floor seen from the right camera
 * @return 3-vector of 3D Robot point coordinates
 */

QVec RobotInnerModel::rightCamFromFloorToRobot(const QVec & p) const
{
	// En el sistema de referencia de la camara
	// (X,Y,Z) = a*(x,y,1); donde Y = h

	//get angles in camara RS
	QVec angles = rightCamera.getAnglesHomogeneous( p );
	//get angles in robot RS
	float tilt = tan(-angles(1) + centralToRightMotor.getRxValue());
	float rightPan = tan(angles(0) + rightMotorToRightCamera.getRyValue());
	Q_ASSERT( tilt < 0);  // we want something below the horizon line
	float h = rightCamToRobot(QVec::zeros(3))(1);  //altura
	float a = h / -tilt;
 	QVec r(3);
 	r(0) = a * rightPan;
 	r(1) = -h;
 	r(2) = a;

	return robotToCentral.inverseTr( centralToRightMotor.inverseTr(rightMotorToRightCamera.inverseTr(r)));
}

//Head-body conversions

/**
 * \brief Converts a 3D point in left Camera reference system to Robot reference system
 * @param p 3-vector of 3D point in Left Camera RS
 * @return 3-vector of 3D point in World RS
 */
QVec RobotInnerModel::leftCamToRobot( const QVec & p) const
{
	return robotToCentral.inverse( centralToLeftMotor.inverse(leftMotorToLeftCamera.inverse( p ) ) ) ;
}

/**
 * \brief Converts a 3D point in third Camera reference system to Robot reference system
 * @param p 3-vector of 3D point in Left Camera RS
 * @return 3-vector of 3D point in World RS
 */
QVec RobotInnerModel::thirdCamToRobot( const QVec & p) const
{
	return robotToCentral.inverse( centralToThirdMotor.inverse(thirdMotorToThirdCamera.inverse( p ) ) ) ;
}

/**
 * \brief Converts a 3D point in right Camera reference system to Robot reference system
 * @param p 3-vector of 3D point in Right Camera RS
 * @return 3-vector of 3D point in Robot RS
 */
QVec RobotInnerModel::rightCamToRobot( const QVec & p) const
{
	return robotToCentral.inverse( centralToRightMotor.inverse(rightMotorToRightCamera.inverse( p ) ) ) ;
}

/**
 * \brief Converts a 3D point in left Camera reference system to World reference system
 * @param p 3-vector of 3D point in Left Camera RS
 * @return 3-vector of 3D point in World RS
 */
QVec RobotInnerModel::leftCamToWorld( const QVec & p) const
{
	return base.robotToWorld ( robotToCentral.inverse( centralToLeftMotor.inverse(leftMotorToLeftCamera.inverse( p ) ) ) );
}

/**
 * \brief Converts a 3D point in third Camera reference system to World reference system
 * @param p 3-vector of 3D point in Third Camera RS
 * @return 3-vector of 3D point in World RS
 */
QVec RobotInnerModel::thirdCamToWorld( const QVec & p) const
{
	return base.robotToWorld ( robotToCentral.inverse( centralToThirdMotor.inverse(thirdMotorToThirdCamera.inverse( p ) ) ) );
}

/**
 * \brief Converts a 3D point in World reference system to Left Camera reference system
 * @param p 3-vector of 3D point in world reference system
 * @return 3-vector of 3D point in left cam reference system
 */
QVec RobotInnerModel::worldToLeftCam( const QVec & p) const
{
	return leftMotorToLeftCamera.direct( centralToLeftMotor.direct( robotToCentral.direct( base.worldToRobot( p ) ) ) );
}

/**
 * \brief Converts a 3D point in World reference system to Third Camera reference system
 * @param p 3-vector of 3D point in world reference system
 * @return 3-vector of 3D point in third cam reference system
 */
QVec RobotInnerModel::worldToThirdCam( const QVec & p) const
{
	return thirdMotorToThirdCamera.direct( centralToThirdMotor.direct( robotToCentral.direct( base.worldToRobot( p ) ) ) );
}

/**
 * \brief Converts a 3D point in World reference system to Right Camera reference system
 * @param p 3D point in world ref system
 * @return 3D point in right cam ref system
 */
QVec RobotInnerModel::worldToRightCam( const QVec & p) const
{
	return rightMotorToRightCamera.direct( centralToRightMotor.direct( robotToCentral.direct( base.worldToRobot( p ) ) ) );
}

 /**
 * \brief Converts a 3D point in right Camera reference system to World reference system
 * @param p 3-vector of 3D point in right Camera RS
 * @return 3-vector of 3D point in World RS
 */
QVec RobotInnerModel::rightCamToWorld( const QVec & p) const
{
	return base.robotToWorld ( robotToCentral.inverse( centralToRightMotor.inverse(rightMotorToRightCamera.inverse( p ) ) ) );
}

/**
 * \brief Converts a 3D point in center of Head reference system to World reference system
 * @param p 3-vector of 3D point in center of Head reference system
 * @return 3-vector of 3D point in World RS
 */
QVec RobotInnerModel::centralToWorld(const QVec & p) const
{
	return base.robotToWorld ( robotToCentral.inverse(( p )));
}

QVec RobotInnerModel::worldToCentral(const QVec & p) const
{
	return robotToCentral.direct(base.worldToRobot( p ));
}


void RobotInnerModel::leftImgToAng(int x, int y, float &pan, float &tilt) const
{
	QVec ray;
	float vx, vy, vz, tiltInc;

	ray = leftCamera.getRay(QVec(QPoint(x,y)));
	vx=ray(0);
	vy=ray(1);
	vz=1.;

	tiltInc=atan2(vy, vz*cos(hState.left.pos)-vx*sin(hState.left.pos));
	tilt=tiltInc+hState.tilt.pos;
	pan=atan2((vx*cos(hState.left.pos)+vz*sin(hState.left.pos))*cos(tiltInc), -vx*sin(hState.left.pos)+vz*cos(hState.left.pos));

}

void RobotInnerModel::rightImgToAng(int x, int y, float &pan, float &tilt) const
{
	QVec ray;
	float vx, vy, vz, tiltInc;

	ray = rightCamera.getRay(QVec(QPoint(x,y)));
	vx=ray(0);
	vy=ray(1);
	vz=1.;

	tiltInc=atan2(vy, vz*cos(hState.right.pos)-vx*sin(hState.right.pos));
	tilt=tiltInc+hState.tilt.pos;
	pan=atan2((vx*cos(hState.right.pos)+vz*sin(hState.right.pos))*cos(tiltInc), -vx*sin(hState.right.pos)+vz*cos(hState.right.pos));

}

void RobotInnerModel::leftAngToImg(float pan, float tilt, int &x, int &y) const
{
	float xTrans, yTrans, zTrans, tiltInc;
	QVec p(3);

	tiltInc=tilt-hState.tilt.pos;
	xTrans=cos(hState.left.pos)*sin(pan) - cos(pan)*cos(tiltInc)*sin(hState.left.pos);
	yTrans=cos(pan)*sin(tiltInc);
	zTrans=sin(pan)*sin(hState.left.pos) + cos(pan)*cos(tiltInc)*cos(hState.left.pos);

	p(0)=xTrans/zTrans; p(1)=yTrans/zTrans; p(2)=1;
	p=leftCamera.project(p);
	x=(int) rint(p(0));
	y=(int) rint(p(1));

}

void RobotInnerModel::rightAngToImg(float pan, float tilt, int &x, int &y) const
{
/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
	float xTrans, yTrans, zTrans, tiltInc;
	QVec p(3);

	tiltInc=tilt-hState.tilt.pos;
	xTrans=cos(hState.right.pos)*sin(pan) - cos(pan)*cos(tiltInc)*sin(hState.right.pos);
	yTrans=cos(pan)*sin(tiltInc);
	zTrans=sin(pan)*sin(hState.right.pos) + cos(pan)*cos(tiltInc)*cos(hState.right.pos);

	p(0)=xTrans/zTrans; p(1)=yTrans/zTrans; p(2)=1;
	p=rightCamera.project(p);
	x=(int) rint(p(0));
	y=(int) rint(p(1));

}


/**
 * \brief Returns true if any of the motors in the stereo head was moving during las update. It is normally used to discard fuzzy frames
 * @return
 */
bool RobotInnerModel::isMoving() const
{
	if ( fabs(bState.x - bStateAnt.x) > 0.5 or fabs(bState.alpha - bStateAnt.alpha) > 0.5  or fabs(bState.z - bStateAnt.z) > 0.01 )
		return true;
	else
		return false;
}

// CLONES RELATED

RobotInnerModel RobotInnerModel::clonFake(const QVec & basePose) const
{
	RobotInnerModel rob( *this );
	RoboCompDifferentialRobot::TBaseState base;
	base.x = basePose(0);
	base.z = basePose(1);
	base.alpha = basePose(2);
	rob.updatePropioception( base , hState );
	return rob;
}

///FUNDAMENTAL RELATED
/**
 * \brief Computes the translation vector that goes from right CRS origin to left CRS origin, defined in the right CRS
 * @return QMat(3,1) translation vector from RCORS to LCORS
 */
QMat RobotInnerModel::rightCamTranslationToLeftCam() const
{
	QMat p1 = rightMotorToRightCamera.inverse( QMat::zeroes(3,1));
	//p1.print("p1");
	QMat p2 = centralToRightMotor.inverse( p1);
	//p2.print("p2");
	QMat p3 = centralToLeftMotor.direct( p2 );
	//p3.print("p3");
	//leftMotorToLeftCamera.getR().print("r");
	//leftMotorToLeftCamera.getTr().print("tR");
	QMat trans = leftMotorToLeftCamera.direct( p3 );
	//qDebug() << "norma" << trans.vectorNormL2();

	//QMat trans = leftMotorToLeftCamera.direct( centralToLeftMotor.direct( centralToRightMotor.inverse( rightMotorToRightCamera.inverse( QMat::zeroes(3,1)))));
	//eftMotorToLeftCamera.print("lmtlc");
	//trans.print("Right cam wrt Left");
	return trans;
}

/**
 * \brief Computes the rotation matrix showing how a 3D point in right cam reference system can be seen in the left cam reference system
 *
 * <p>
 *	If  QVec Q = model->rightCamRotationToLeftCam() * P, then Q is P seen from the left reference system
 * </p>
 * @return QMat(3,3) rotation matrix
 */
QMat RobotInnerModel::rightCamRotationToLeftCam() const
{
	QMat r1 = leftMotorToLeftCamera.getR().transpose();
	//r1.print("r1");
	QMat r2 = centralToLeftMotor.getR().transpose();
	//r2.print("r2");
	QMat r3 = centralToRightMotor.getR();
	//r3.print("r3");
	QMat r4 = rightMotorToRightCamera.getR();
	//r4.print("r4");
	QMat rot = r1*(r2*(r3*r4));

/*	QMat rot = leftMotorToLeftCamera.getR().transpose() * centralToLeftMotor.getR().transpose() * centralToRightMotor.getR() *
	 rightMotorToRightCamera.getR();*/
	return rot;
}

/**
 * \brief Returns current copy of fundamenta matrix as a float h[3][3] array
 * @param h[][] [3][3] preallocates array of floats to contain de fundamental matrix
 */
void RobotInnerModel::getFundamental(float h[3][3]) const
{
	h[0][0] = fundamental(0,0);
	h[0][1] = fundamental(0,1);
	h[0][2] = fundamental(0,2);
	h[1][0] = fundamental(1,0);
	h[1][1] = fundamental(1,1);
	h[1][2] = fundamental(1,2);
	h[2][0] = fundamental(2,0);
	h[2][1] = fundamental(2,1);
	h[2][2] = fundamental(2,2);
}


/**
 * \brief Computes de 3D triangulation of two correspondent image points in robot reference system
 * @param left 2D image reference system point
 * @param right 2D image reference system point
 * @return 3D point in robot(base) reference system
 */
QVec RobotInnerModel::compute3DPointToRobot(const QVec & left, const QVec & right)
{
	T detA, a, b, c;

	RMat::QVec pI = (centralToLeftMotor.getR().transpose() * leftMotorToLeftCamera.getR().transpose()) * leftCamera.getRayHomogeneous( left );
	RMat::QVec pD = (centralToRightMotor.getR().transpose() * rightMotorToRightCamera.getR().transpose()) * rightCamera.getRayHomogeneous( right );

	RMat::QVec n = QVec::vec3( pI(1)-pD(1) , -pI(0)+pD(0) , pI(0)*pD(1)-pD(0)*pI(1) );

	RMat::QMat A(3,3);
	A(0,0)=pI(0);		A(0,1)=-pD(0);	A(0,2)=n(0);
	A(1,0)=pI(1);		A(1,1)=-pD(1);	A(1,2)=n(1);
	A(2,0)=1;			A(2,1)=-1;		A(2,2)=n(2);

	detA = A(0,0)*(A(1,1)*A(2,2)-A(1,2)*A(2,1))-A(0,1)*(A(1,0)*A(2,2)-A(1,2)*A(2,0))+A(0,2)*(A(1,0)*A(2,1)-A(1,1)*A(2,0));

	float baseLine = getBaseLine();
	a=baseLine*(-pD(1)*n(2)+n(1))/detA;
	b=baseLine*(pI(1)*n(2)-n(1))/detA;
	c=baseLine*(-pI(1)+pD(1))/detA;

	QVec res(3);
	res(0) = (a*pI(0)-(baseLine/2.))+(c*n(0))/2.;
	res(1) = a*pI(1) + c*n(1)/2.;
	res(2) = a*pI(2) + c*n(2)/2.;

	return robotToCentral.inverse(res);
}

/**
 * \brief Computes de 3D triangulation of two correspondent image points in world reference system
 * @param left 2D image reference system point
 * @param right 2D image reference system point
 * @return 3D point in world(reference system
 */
QVec RobotInnerModel::compute3DPoint(const QVec & left, const QVec & right)
{
	return robotToWorld(compute3DPointToRobot(left, right));
}


/**
 * \brief Inverse projection. Returns a 3D vector in WRS as the point along optical ray throug (i,j) and at length dist in left camera
 * @param i image coordinate
 * @param j image coordinate
 * @param dist distance along optical ray
 * @return 3D vector in WRS
 */
QVec RobotInnerModel::getWorld3DPointAlongRayGoingOutLeftCamera(int i, int j, float dist) const
{
	QVec ang = this->leftCamera.getAnglesHomogeneous(QVec::vec2(i,j));
	ang(0)=tan(ang(0));
	ang(1)=tan(ang(1));
	QVec punto=ang* dist;
	return this->leftCamToWorld( punto );
}

/**
 * \brief Inverse projection. Returns a 3D vector in WRS as the point along optical ray throug (i,j) and at length dist in right camera
 * @param i image coordinate
 * @param j image coordinate
 * @param dist distance along optical ray
 * @return 3D vector in WRS
 */
QVec RobotInnerModel::getWorld3DPointAlongRayGoingOutRightCamera(int i, int j, float dist) const
{
	QVec ang = this->rightCamera.getAnglesHomogeneous(QVec::vec2(i,j));
	ang(0)=tan(ang(0));
	ang(1)=tan(ang(1));
	QVec punto=ang* dist;
	return this->rightCamToWorld( punto );

}

/**
 * \brief Returns a 3D vector (A,B,C) containing the horizon line for the left image in the form 'Ax + By + C = 0'.
 *
 * <p>
 * Returns a 3D vector (A,B,C) containing the horizon line in the form Ax + By + C = 0. For general lines, it will also work as 'y = Ax + C' (not for vertical lines, which are a very rare case).
 * You can check B to know if the returned vector is a regular line:
 * </p>
 * <p>
 * QVec horizon = innerModel->getImageHorizonLine();
 * <br>
 * if (horizon(1) == 0) printf("Vertical horizon.\n");
 * <br>
 * else printf("Regular horizon.\n");
 * </p>
 *
 * @pre Intrinsic and extrinsic parameters are correctly set for the left camera.
 *
 */
QVec RobotInnerModel::getLeftImageHorizonLine(float heightOffset) const
{
	QVec proj1 = projectFromRobotToLeftCam(QVec::vec3(-500000., 0., 10000000.));
	QVec proj2 = projectFromRobotToLeftCam(QVec::vec3( 500000., 0., 10000000.));

	double dx=proj2(0)-proj1(0);
	double dy=proj2(1)-proj1(1);

	if (abs(dx) <= 1)
	{
		if (abs(dy) <= 1) qFatal("Degenerated camera");
		qWarning("Vertical horizon :s ?");
		return QVec::vec3(-1, 0, proj1(0));
	}
	else return QVec::vec3(dy/dx, -1, proj1(1)-(dy*proj1(0)/dx)+heightOffset);
}

/**
 * \brief Returns a 3D vector (A,B,C) containing the horizon line for the third image in the form 'Ax + By + C = 0'.
 *
 * <p>
 * Returns a 3D vector (A,B,C) containing the horizon line in the form Ax + By + C = 0. For general lines, it will also work as 'y = Ax + C' (not for vertical lines, which are a very rare case).
 * You can check B to know if the returned vector is a regular line:
 * </p>
 * <p>
 * QVec horizon = innerModel->getImageHorizonLine();
 * <br>
 * if (horizon(1) == 0) printf("Vertical horizon.\n");
 * <br>
 * else printf("Regular horizon.\n");
 * </p>
 *
 * @pre Intrinsic and extrinsic parameters are correctly set for the third camera.
 *
 */
QVec RobotInnerModel::getThirdImageHorizonLine(float heightOffset) const
{
	QVec proj1 = projectFromRobotToLeftCam(QVec::vec3(-500000., 0., 10000000.));
	QVec proj2 = projectFromRobotToLeftCam(QVec::vec3( 500000., 0., 10000000.));
	double dx=proj2(0)-proj1(0);
	double dy=proj2(1)-proj1(1);

	if (abs(dx) <= 1)
	{
		if (abs(dy) <= 1) qFatal("Degenerated camera");
		qWarning("Vertical horizon :s ?");
		return QVec::vec3(-1, 0, proj1(0));
	}
	else return QVec::vec3(dy/dx, -1, proj1(1)-(dy*proj1(0)/dx)+heightOffset);
}

/**
 * \brief Returns a 3D vector (A,B,C) containing the horizon line for the left image in the form 'Ax + By + C = 0'.
 *
 * <p>
 * Returns a 3D vector (A,B,C) containing the horizon line in the form Ax + By + C = 0. For general lines, it will also work as 'y = Ax + C' (not for vertical lines, which are a very rare case).
 * You can check B to know if the returned vector is a regular line:
 * </p>
 * <p>
 * QVec horizon = innerModel->getImageHorizonLine();
 * <br>
 * if (horizon(1) == 0) printf("Vertical horizon.\n");
 * <br>
 * else printf("Regular horizon.\n");
 * </p>
 *
 * @pre Intrinsic and extrinsic parameters are correctly set for the right camera.
 *
 */
QVec RobotInnerModel::getRightImageHorizonLine(float heightOffset) const
{
	QVec proj1 = projectFromRobotToRightCam(QVec::vec3(-500000., 0., 10000000.));
	QVec proj2 = projectFromRobotToRightCam(QVec::vec3( 500000., 0., 10000000.));
	double dx=proj2(0)-proj1(0);
	double dy=proj2(1)-proj1(1);

	if (abs(dx) <= 1)
	{
		if (abs(dy) <= 1) qFatal("Degenerated camera");
		qWarning("Vertical horizon :s ?");
		return QVec::vec3(-1, 0, proj1(0));
	}
	else
		return QVec::vec3(dy/dx, -1, proj1(1)-(dy*proj1(0)/dx)+heightOffset);
}

/**
 * \brief Computes current frustrum for left camera and stores it in this->leftFrustrum
 *
 * <p>
 * Frustrum is a six plane construction. We need three points for each one.
 * Common point is camera central points. The two others for the plane are computed from camera angles
 * One of the most common ways to define a plane is with the following equation:
 * Ax + By + Cz + D = 0
 * Assuming three points p0, p1, and p2 the coefficients A, B, C and D can be computed as follows:
 * Compute vectors v = p1 - p0, and u = p2 - p0
 * Compute n = v x u (cross product)
 * Normalize n
 * Assuming n = (xn,yn,zn) is the normalized normal vector then
 *		A = xn
 *		B = yn
 *		C = zn
 * To compute the value of D we just use the equation above, hence -D = Ax + By + Cz
 * From the above point, and replacing (x,y,z) for a point in the plane (for instance p0), we get D = - n . p0 (dot product).
 * Normals should point inwards
 */
void RobotInnerModel::computeLeftFrustrum()
{
	QVec p1(3), p2(3), p3(3);
	int MIN_DIST = 300; //mm GET FROM CMAMERA PARAMETERS
	int MAX_DIST = 4000; //mm

	//Center
	QVec pc = this->leftCamToWorld( QVec::zeros(3));

	//left
	p1 = this->getWorld3DPointAlongRayGoingOutLeftCamera(0,0,MAX_DIST);
	p2 = this->getWorld3DPointAlongRayGoingOutLeftCamera(0,240,MAX_DIST);
	this->frustrumLeft.left.n = ((p1-pc)^(p2-pc)).normalize();
	this->frustrumLeft.left.d = -(this->frustrumLeft.left.n * pc);

	//Right
	p1 = this->getWorld3DPointAlongRayGoingOutLeftCamera(320,0,MAX_DIST);
	p2 = this->getWorld3DPointAlongRayGoingOutLeftCamera(320,240,MAX_DIST);
	this->frustrumLeft.right.n = ((p2-pc)^(p1-pc)).normalize();
	this->frustrumLeft.right.d = -(this->frustrumLeft.right.n * pc);

	//Top
	p1 = this->getWorld3DPointAlongRayGoingOutLeftCamera(0,0,MAX_DIST);
	p2 = this->getWorld3DPointAlongRayGoingOutLeftCamera(320,0,MAX_DIST);
	this->frustrumLeft.top.n = ((p2-pc)^(p1-pc)).normalize();
	this->frustrumLeft.top.d = -(this->frustrumLeft.top.n * pc);

	//Down
	p1 = this->getWorld3DPointAlongRayGoingOutLeftCamera(0,240,MAX_DIST);
	p2 = this->getWorld3DPointAlongRayGoingOutLeftCamera(320,240,MAX_DIST);
	this->frustrumLeft.down.n = ((p1-pc)^(p2-pc)).normalize();
	this->frustrumLeft.down.d = -(this->frustrumLeft.down.n * pc);

 	//nEAR
	p1 = this->getWorld3DPointAlongRayGoingOutLeftCamera(0,0,MIN_DIST);
	p2 = this->getWorld3DPointAlongRayGoingOutLeftCamera(320,0,MIN_DIST);
	p3 = this->getWorld3DPointAlongRayGoingOutLeftCamera(320,240,MIN_DIST);
	this->frustrumLeft.near.n = ((p3-p1)^(p2-p1)).normalize();
	this->frustrumLeft.near.d = -(this->frustrumLeft.near.n * pc);

	//far
	p1 = this->getWorld3DPointAlongRayGoingOutLeftCamera(0,0,MAX_DIST);
	p2 = this->getWorld3DPointAlongRayGoingOutLeftCamera(320,0,MAX_DIST);
	p3 = this->getWorld3DPointAlongRayGoingOutLeftCamera(320,240,MAX_DIST);
	this->frustrumLeft.far.n = ((p3-p1)^(p2-p1)).normalize();
	this->frustrumLeft.far.d = -(this->frustrumLeft.far.n * pc);
}

/**
 * Returns true if a 3D point in WRS is inside left frustrum as computed by computeLeftFrustrum()
 * @param p 3D point in WRS
 * @return true if inside, false if outside
 */
bool RobotInnerModel::check3DPointInsideLeftFrustrum(const QVec & p)
{
	if (p * this->frustrumLeft.left.n + this->frustrumLeft.left.d > 0. and
	  p * this->frustrumLeft.right.n + this->frustrumLeft.right.d > 0. and
	  p * this->frustrumLeft.far.n + this->frustrumLeft.far.d > 0. and
	  p * this->frustrumLeft.near.n + this->frustrumLeft.near.d > 0. and
	  p * this->frustrumLeft.down.n + this->frustrumLeft.down.d > 0. and
	  p * this->frustrumLeft.top.n + this->frustrumLeft.top.d > 0.
	  )
		return true;
	else
		return false;
}

RoboCompCamMotion::THeadState RobotInnerModel::headnuevoToantiguo(RoboCompCommonHead::THeadState head)
{
	  RoboCompCamMotion::THeadState aux;
	  RoboCompCamMotion::TMotorState motor;

	  aux.isMoving = head.isMoving;

	  //TILT
	  motor.pos = head.motorsState["tilt"].pos;
	  motor.vel = head.motorsState["tilt"].vel;
	  motor.power = head.motorsState["tilt"].power;
	  motor.p = head.motorsState["tilt"].p;
	  motor.v = head.motorsState["tilt"].v;
	  motor.isMoving = head.motorsState["tilt"].isMoving;

	  aux.tilt = motor;
	  aux.leftTilt = motor;
	  aux.rightTilt = motor;

	  //NECK
	  motor.pos = head.motorsState["neck"].pos;
	  motor.vel = head.motorsState["neck"].vel;
	  motor.power = head.motorsState["neck"].power;
	  motor.p = head.motorsState["neck"].p;
	  motor.v = head.motorsState["neck"].v;
	  motor.isMoving = head.motorsState["neck"].isMoving;

	  aux.neck = motor;

	  //LEFT PAN
	  motor.pos = head.motorsState["leftPan"].pos;
	  motor.vel = head.motorsState["leftPan"].vel;
	  motor.power = head.motorsState["leftPan"].power;
	  motor.p = head.motorsState["leftPan"].p;
	  motor.v = head.motorsState["leftPan"].v;
	  motor.isMoving = head.motorsState["leftPan"].isMoving;

	  aux.left = motor;

	  //RIGHT PAN
	  motor.pos = head.motorsState["rightPan"].pos;
	  motor.vel = head.motorsState["rightPan"].vel;
	  motor.power = head.motorsState["rightPan"].power;
	  motor.p = head.motorsState["rightPan"].p;
	  motor.v = head.motorsState["rightPan"].v;
	  motor.isMoving = head.motorsState["rightPan"].isMoving;

	  aux.right = motor;

	  return aux;
}

RoboCompCamMotion::THeadState RobotInnerModel::headnuevoToantiguo(RoboCompJointMotor::MotorStateMap head)
{
	  RoboCompCamMotion::THeadState aux;
	  RoboCompCamMotion::TMotorState motor;


	  aux.isMoving = head["tilt"].isMoving or head["neck"].isMoving or head["leftPan"].isMoving or head["rightPan"].isMoving;

	  //TILT
	  motor.pos = head["tilt"].pos;
	  motor.vel = head["tilt"].vel;
	  motor.power = head["tilt"].power;
	  motor.p = head["tilt"].p;
	  motor.v = head["tilt"].v;
	  motor.isMoving = head["tilt"].isMoving;

	  aux.tilt = motor;
	  aux.leftTilt = motor;
	  aux.rightTilt = motor;

	  //NECK
	  motor.pos = head["neck"].pos;
	  motor.vel = head["neck"].vel;
	  motor.power = head["neck"].power;
	  motor.p = head["neck"].p;
	  motor.v = head["neck"].v;
	  motor.isMoving = head["neck"].isMoving;

	  aux.neck = motor;

	  //LEFT PAN
	  motor.pos = head["leftPan"].pos;
	  motor.vel = head["leftPan"].vel;
	  motor.power = head["leftPan"].power;
	  motor.p = head["leftPan"].p;
	  motor.v = head["leftPan"].v;
	  motor.isMoving = head["leftPan"].isMoving;

	  aux.left = motor;

	  //RIGHT PAN
	  motor.pos = head["rightPan"].pos;
	  motor.vel = head["rightPan"].vel;
	  motor.power = head["rightPan"].power;
	  motor.p = head["rightPan"].p;
	  motor.v = head["rightPan"].v;
	  motor.isMoving = head["rightPan"].isMoving;

	  aux.right = motor;

	  return aux;
}


