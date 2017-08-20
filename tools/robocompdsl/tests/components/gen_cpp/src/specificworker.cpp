/*
 *    Copyright (C)2017 by YOUR NAME HERE
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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	//computeCODE
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}

	if(!substractBuffer.isEmpty()){
		uint id;
		int num1;
		int num2;
		std::forward_as_tuple(id, std::tie(num1,num2)) = substractBuffer.pop();
		
		int result = num1 - num2 ;
		substractBuffer.setFinished(id, std::make_tuple(result));
	}

	if(!printmsgBuffer.isEmpty()){
		uint id;
		string message;
		std::forward_as_tuple(id, std::tie(message)) = printmsgBuffer.pop();
		cout<<message<<endl;
	}

	if(!sumBuffer.isEmpty()){
		uint id;
		int num1;
		int num2;
		std::forward_as_tuple(id, std::tie(num1,num2)) = sumBuffer.pop();
		int ret = num2 + num1;
		sumBuffer.setFinished(id, std::make_tuple(ret));
	}

	if(!divideBuffer.isEmpty()){
		uint id;
		int divident;
		int divisor;
		std::forward_as_tuple(id, std::tie(divident,divisor)) = divideBuffer.pop();
		int ret = divident/divisor ;
		int reminder = divident%divisor;
		divideBuffer.setFinished(id, std::make_tuple(ret, reminder));
	}


}



