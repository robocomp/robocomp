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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

// #include <ipp.h>
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>
#include <ui_guiDlg.h>
#include <DifferentialRobot.h>
#include <RCISMousePicker.h>
#include <Camera.h>
#include <Laser.h>
#include <RGBD.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 16

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

/**
       \brief
       @author authorname
*/
using namespace RoboCompDifferentialRobot;
using namespace RoboCompCamera;
using namespace RoboCompLaser;
using namespace RoboCompRCISMousePicker;

class GenericWorker : public QMainWindow, public Ui_guiDlg
{
	Q_OBJECT
public:
	GenericWorker ( MapPrx& mprx );
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod ( int p );
	RCISMousePickerPrx rcis_mousepicker_proxy;
	QMutex *mutex;                //Shared mutex with servant
	QTimer timer;
	
protected:
	int Period;
	//MapPrx proxies;

public slots:
	virtual void compute() = 0;

signals:
	void kill();
};

#endif
