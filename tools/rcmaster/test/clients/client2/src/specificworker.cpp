/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(MapPrx& mprx, Mapiface& miface) : GenericWorker(mprx, miface)
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



	
	timer.start(Period);
	

	return true;
}

void SpecificWorker::waitforComp(::IceProxy::Ice::Object* proxy, string interfaceName)
{

	timer.stop();
	sleep(3);
	bool flag = false;
	for(auto const &iface : ifaces) 
	{
		if( iface.second.alias.compare(interfaceName)==0)
		{
			flag = true;
			break;
		}
	}
	if (flag==false)
	{
		timer.start(Period);
		throw("interface "+interfaceName+" dosent exist");
	}

	string host;
	if (interfaceName.compare("test")==0){
		testPrx proxyf = (*(testPrx*)proxy);
		host = proxyf->ice_toString();
	}

	host = host.substr(host.find("-h")+3,(host.find("-p")-host.find("-h")-3));

	Ice::CommunicatorPtr ic;
    ic = Ice::initialize();
	string compName = ifaces[interfaceName].comp;//neep compMap

	while (true)
	{
		try
		{
			interfaceList interfaces = rcmaster_proxy->getComp(compName,host);
			string port = "0";
			for (auto const &iface :interfaces)
			{
				if (iface.name.compare("test")==0)				{
					port = std::to_string(iface.port);
					string proxyStr = "test:"+iface.protocol+" -h "+host+" -p "+port;
					test_proxy = testPrx::uncheckedCast( ic->stringToProxy( proxyStr ) );
				}
			}
		}
		catch (const ComponentNotFound& ex)
		{
			cout<< "waiting for "<< compName<<endl;
			sleep(3);
			continue;
		}
		catch  (const Ice::SocketException& ex)
		{
			cout<< "waiting for "<< compName<<endl;
			sleep(3);
			continue;
		}
		catch (const Ice::Exception& ex)
		{
			cout<<"Cannot connect to the remote object "<<compName;
			sleep(3);
			continue;
		}		
		timer.start(Period);
		break;
	}

}

void SpecificWorker::compute()
{
//computeCODE
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch ( const Ice::SocketException& ex)
//	{
//		waitforComp((::IceProxy::Ice::Object*)(&camera_proxy),"camera");
//	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
// 		try

}


void SpecificWorker::printmsg(const string &message)
{
//implementCODE

}






