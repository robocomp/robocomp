/*
 *    Copyright (C)2020 by YOUR NAME HERE
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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
#include <QStateMachine>
#include <QState>
#include <CommonBehavior.h>

#include <AprilBasedLocalization.h>
#include <AprilTags.h>
#include <CameraSimple.h>
#include <GenericBase.h>
#include <HandDetection.h>
#include <JointMotor.h>
#include <RGBD.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompAprilBasedLocalization;
using namespace RoboCompAprilTags;
using namespace RoboCompCameraSimple;
using namespace RoboCompGenericBase;
using namespace RoboCompHandDetection;
using namespace RoboCompJointMotor;
using namespace RoboCompRGBD;

typedef map <string,::IceProxy::Ice::Object*> MapPrx;


class GenericWorker :
#ifdef USE_QTGUI
	public QDialog, public Ui_guiDlg
#else
	public QObject
 #endif
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;


	CameraSimplePrx camerasimple_proxy;
	RGBDPrx rgbd_proxy;
	AprilBasedLocalizationPrx aprilbasedlocalization_pubproxy;

	virtual int HandDetection_addNewHand(const int expectedHands, const TRoi &roi) = 0;
	virtual Hands HandDetection_getHands() = 0;
	virtual int HandDetection_getHandsCount() = 0;
	virtual void AprilTags_newAprilTag(const tagsList &tags) = 0;
	virtual void AprilTags_newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState) = 0;

protected:
	//State Machine
	QStateMachine defaultMachine;

	QState *computeState;
	QState *initializeState;
	QFinalState *finalizeState;

	//-------------------------

	QTimer timer;
	int Period;

private:


public slots:
	//Slots funtion State Machine
	virtual void sm_compute() = 0;
	virtual void sm_initialize() = 0;
	virtual void sm_finalize() = 0;

	//-------------------------
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
	//Signals for State Machine
	void t_initialize_to_compute();
	void t_compute_to_compute();
	void t_compute_to_finalize();

	//-------------------------
};

#endif
