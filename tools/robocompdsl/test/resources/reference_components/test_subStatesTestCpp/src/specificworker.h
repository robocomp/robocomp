/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void initialize(int period);
	//Specification slot methods State Machine
	void sm_two();
	void sm_three();
	void sm_four();
	void sm_one();
	void sm_five();
	void sm_test2sub1();
	void sm_test2sub2();
	void sm_test2sub21();
	void sm_test2sub22();
	void sm_test3sub1();
	void sm_test3sub2();
	void sm_test3sub3();
	void sm_test4sub2();
	void sm_test4sub1();

	//--------------------
private:
	std::shared_ptr<InnerModel> innerModel;

};

#endif
