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
#ifndef GENERICMONITOR_H
#define GENERICMONITOR_H

#include <Ice/Ice.h>
#include <QtCore>
#include "genericworker.h"
#include "config.h"
#include <qlog/qlog.h>
#include <CommonBehavior.h>

/**
       \brief
       @author authorname
*/
class GenericMonitor : public QThread
{
Q_OBJECT

public:
	GenericMonitor(GenericWorker *_worker, Ice::CommunicatorPtr _communicator);
	virtual ~GenericMonitor();


    //CommonBehavior
	int getPeriod();
	void setPeriod(int period);
	void killYourSelf();
	int timeAwake();
	RoboCompCommonBehavior::ParameterList getParameterList();
	void setParameterList(RoboCompCommonBehavior::ParameterList l);
	RoboCompCommonBehavior::State getState();

	void readPConfParams(RoboCompCommonBehavior::ParameterList &params);
	virtual void readConfig(RoboCompCommonBehavior::ParameterList &params ) = 0;
	virtual void run() = 0;
	virtual void initialize() = 0;

protected:
	int period;
	GenericWorker *worker;
	Ice::CommunicatorPtr communicator;
	QTime initialTime;
	RoboCompCommonBehavior::ParameterList config_params;
	RoboCompCommonBehavior::State state;

	virtual bool sendParamsToWorker(RoboCompCommonBehavior::ParameterList params) = 0;
	virtual bool checkParams(RoboCompCommonBehavior::ParameterList l) = 0;

	bool configGetString(const std::string prefix, const std::string name, std::string &value, const std::string default_value, QStringList *list = NULL);
public:
	static bool configGetString(Ice::CommunicatorPtr communicator, const std::string prefix, const std::string name, std::string &value, const std::string default_value, QStringList *list=NULL);

signals:
	void kill();
	void initializeWorker(int);
};

#endif // GENERICMONITOR_H
