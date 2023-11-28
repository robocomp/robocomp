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

/**
	\brief
	@author authorname
*/

${agmagent_comment}


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
${dsr_includes}

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(${constructor_proxies}, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	${implements_method_definitions}

	${subscribes_method_definitions}

public slots:
	${compute}
	int startup_check();
	void initialize(int period);
	${statemachine_methods_definitions}
	${dsr_slots}
private:
	${agm_attributes}
	${dsr_attributes}
	bool startup_check_flag;

};

#endif
