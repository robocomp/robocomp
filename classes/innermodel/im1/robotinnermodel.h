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
#ifndef ROBOTINNERMODEL_H
#define ROBOTINNERMODEL_H

#include <QtCore>
#include <QMat/QMatAll>

#include <DifferentialRobot.h>
#include <CamMotion.h>
#include <CommonHead.h>
#include <JointMotor.h>
#include <Laser.h>

using namespace RMat;

/**
 *\brief This class is holds a copy of the kinematics and dimensions of the robot parts and joints. It supplies methods to access coordinate conversion among parts.
 * The state of the robot is updated calling specific methods that copy BaseComp and CamMotion state structures.
 * @author Robolab staff
 *
 * Currently uses 4x4 matrices to build open kinematic chains starting from World's system of reference. These matrices are specialized QMat matrices with ad hoc methods
 * added to facilitate access to explicit rotation angles and translation values.
 *
 */
class RobotInnerModel
{
public:

	//auxiliar
	RoboCompCamMotion::THeadState headnuevoToantiguo(RoboCompCommonHead::THeadState head);
	RoboCompCamMotion::THeadState headnuevoToantiguo(RoboCompJointMotor::MotorStateMap head);


	// Constructors
	/** Default constructor. Properties must be set manually, see RobotInnerModel::init() . */
	RobotInnerModel();

	/** Constructor by name: Set the properties of one of the hardcoded configurations in robotinnermodel.cpp . */
	RobotInnerModel(QString name);

	/** Copy constructor. */
	RobotInnerModel(const RobotInnerModel & rob);   //Quitar para que no se use por ref
	~RobotInnerModel();

	/** Initialization for one of the hardcoded configurations. */
	void init(const QString & rob);

	// Updating methods
	void updatePropioception( const RoboCompCamMotion::THeadState & hState_ );
	void updatePropioception( const RoboCompCommonHead::THeadState & hState_ );
	void updatePropioception( const RoboCompDifferentialRobot::TBaseState & bState_);
	void updatePropioception(const float x, const float z, const float alpha);
	void updatePropioception( const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompCamMotion::THeadState & hState_ );
	void updatePropioception( const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompCommonHead::THeadState & hState_ );
	void updatePropioception( const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompJointMotor::MotorStateMap & hState_ );
	void updatePropioception( const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompCamMotion::THeadState & hState_ , const RoboCompLaser::TLaserData & lData);
	void updatePropioception( const RoboCompDifferentialRobot::TBaseState & bState_, const RoboCompCommonHead::THeadState & hState_ , const RoboCompLaser::TLaserData & lData);
	void updateIncrementalPropioception( float incAng, float incAdv);

	// Base conversions
	/**
	* Converts 3D coordinates of points in Robot reference system to World reference system
	* @param p 3-vector of 3D point in Robot reference system
	* @return 3-vector of 3D point en World reference system
	*/
	inline QVec robotToWorld (const QVec & p) const { return base.inverse( p ); }
	inline QVec robotInWorld() const { return base.inverse(QVec::zeros(3)); }
	/**
	 * Converts 3D coordinates of points in World reference system to Robot reference system
	* @param p 3-vector of 3D point in World reference system
	* @return 3-vector of 3D point en Robot reference system
	*/
	inline QVec worldToRobot (const QVec & p) const { return base.direct( p ); }

	// Head conversions
	QMat rightCamRotationToLeftCam() const;
	QMat rightCamTranslationToLeftCam() const;
	QVec rightCamToRobot ( const QVec & p ) const;
	QVec leftCamToRobot ( const QVec & p ) const;
	QVec thirdCamToRobot ( const QVec & p ) const;
	QVec rightCamToWorld ( const QVec & p ) const;
	QVec leftCamToWorld ( const QVec & p ) const;
	QVec thirdCamToWorld ( const QVec & p ) const;
	QVec centralToWorld ( const QVec & p ) const;
	QVec worldToCentral ( const QVec & p ) const;
	QVec leftCamPlusDepthToWorld(const QVec & p ) const;
	QVec rightCamPlusDepthToWorld(const QVec & p) const;
	QVec rightCamFromFloorToRobot(const QVec & p) const;
	QVec leftCamFromFloorToWorld (const QVec &p ) const;
	QVec thirdCamFromFloorToWorld (const QVec &p ) const;
	QVec rightCamFromFloorToWorld (const QVec &p ) const;
	QVec leftCamFromFloorToRobot(const QVec & p) const;
	QVec thirdCamFromFloorToRobot(const QVec & p) const;
	QVec worldToLeftCam ( const QVec & p ) const;
	QVec worldToThirdCam ( const QVec & p ) const;
	QVec worldToRightCam ( const QVec & p ) const;
	void leftImgToAng(int x, int y, float &pan, float &tilt) const;
	void rightImgToAng(int x, int y, float &pan, float &tilt) const;
	void leftAngToImg(float pan, float tilt, int &x, int &y) const;
	void rightAngToImg(float pan, float tilt, int &x, int &y) const;


	// Proyections
	QVec projectFromRobotToLeftCam (const QVec & p) const;
	QVec projectFromWorldToLeftCam (const QVec & p) const;
	QVec projectFromRobotToThirdCam (const QVec & p) const;
	QVec projectFromWorldToThirdCam (const QVec & p) const;
	QVec projectFromRobotToRightCam (const QVec & p) const;
	QVec projectFromWorldToRightCam (const QVec & p) const;

	// Getters for model parameters
	QVec getBaseOdometry() const;
	QVec getBaseOdometryAnt() const;
	float getLaserHeight() const               { return laser.getTr()(1); }
	inline QVec getBaseCoor() const            { QVec p(3); p(0) = bState.x; p(1) = 0.; p(2) = 	bState.z; return p; }
	inline T getBaseAngle() const              { return bState.alpha; }
	inline T getBaseX() const                  { return bState.x; }
	inline T getBaseZ() const                  { return bState.z; }
	inline T getBaseSpeedAdv() const           { return bState.adv; }
	inline T getBaseSpeedRot() const           { return bState.rot; }
	inline float getRadius()                   { return robotRadius; }
	float getFocal()                           { return leftCamera.getFocal(); }
	QVec getGeometricCenter() const            { return geometricCenter; };
	int getLeftCamWidth() const                { return leftCamera.getWidth(); }
	int getLeftCamHeight() const               { return leftCamera.getHeight(); }
	int getLeftCamSize() const                 { return leftCamera.getSize(); }
	void getFundamental(float h[3][3]) const;
	QMat getFundamental()	const              { return fundamental; }
	inline QVec getHeadAngles()	const          { return QVec::vec3(hState.left.pos, hState.right.pos, hState.tilt.pos); }
	inline T getBaseLine() const               { return rightCamTranslationToLeftCam().vectorNormL2(); }

	// Stereo computations
	QVec compute3DPoint(const QVec & left, const QVec & right);
	QVec compute3DPointToRobot(const QVec & left, const QVec & right);

	// Setters for model parameters
	void setFocal(const int f)                                     { leftCamera.setFocal( f); rightCamera.setFocal( f ); }

	// Laser stuff
	QVec laserToWorld(const QVec &p) const;
	QVec laserToWorld( float r, float alpha) const;
	QVec laserToWorld( int i) const;
	QVec innerLaserToWorld( int i) const;
	QVec innerLaserToBase( int i) const;
	QVec worldToLaser( const QVec & world) const;
	QVec laserToBase( int i) const;
	QVec laserToBase( float dist, float angle) const;

	bool isMoving() const ;

	// Clonning
	RobotInnerModel clonFake( const QVec &basePose) const;

	// Misc
	QVec getLeftImageHorizonLine(float offset=0.) const;
	QVec getThirdImageHorizonLine(float offset=0.) const;
	QVec getRightImageHorizonLine(float offset=0.) const;
	QVec getWorld3DPointAlongRayGoingOutLeftCamera(int i,int j,float dist) const;
	QVec getWorld3DPointAlongRayGoingOutRightCamera(int i,int j,float dist) const;

	///Data. Should be private
	///We have to copy all these when fakeClonning the robot

	// Overall structure of the robot coded as joint's matrices
	Cam leftCamera, rightCamera, thirdCamera;           // Camera matrices
	QExtrinsics robotToCentral;                         // From robot origin to head origin (midpoint between cameras)
	QExtrinsics centralToLeftMotor;                     // From central to left motor
	QExtrinsics centralToThirdMotor;                     // From central to third motor
	QExtrinsics centralToRightMotor;                    // From central to right motor
	QExtrinsics leftMotorToLeftCamera;                  // From left motor to left camera
	QExtrinsics thirdMotorToThirdCamera;                  // From third motor to third camera
	QExtrinsics rightMotorToRightCamera;
	MovingRobot base;
	QExtrinsics laser;
	QFundamental fundamental;
	QEssential	essential;
	QVec geometricCenter;               //Position of geometric center wrt origin

	//Frustrum
	struct TPlano { QVec n; float d; };
	struct TFrustrum { TPlano left; TPlano top; TPlano right; TPlano down; TPlano near; TPlano far;};
	TFrustrum frustrumLeft, frustrumThird, frustrumRight;
	void computeLeftFrustrum();
	bool check3DPointInsideLeftFrustrum( const QVec & p);
	void computeRightFrustrum();
	bool check3DPointInsideRightFrustrum( const QVec & p);

	//State
	RoboCompCamMotion::THeadState hState, hStateAnt;

private:
	RoboCompDifferentialRobot::TBaseState bState;
	RoboCompDifferentialRobot::TBaseState bStateAnt;

	RoboCompLaser::TLaserData laserData;
	RoboCompLaser::LaserConfData laserConf;

	float robotRadius;

};




#endif
