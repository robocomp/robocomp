/*
 *    Copyright (C) ${year} by YOUR NAME HERE
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
${gui_includes}
${statemachine_includes}
#include <CommonBehavior.h>

${interfaces_includes}
${agm_includes}


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
${namespaces}

${ice_proxies_map}

${agm_behaviour_parameter_struct}

${ros_publishes_classes}
${ros_requires_classes}
class GenericWorker : ${inherited_object}
{
Q_OBJECT
public:
	GenericWorker(${constructor_proxies});
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;
	${agm_methods}


	${create_proxies}

	${implements}
	${subscribes}

protected:
	${statemachine_creation}

	QTimer timer;
	int Period;
	${ros_interfaces_creation}
	${agm_attributes_creation}

private:


public slots:
	${statemachine_slots}
	${virtual_compute}
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
	${statemachine_signals}
};

#endif
