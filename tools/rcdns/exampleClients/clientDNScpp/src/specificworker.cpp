/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
/*
 * 		examples to use method
 * 
 */
	requestRealPort();
	
	RoboCompRCDNS::ipInfo hostInfo;
	hostInfo.publicIP = "158.49.247.2";
	hostInfo.privateIP = "192.168.1.2";
	hostInfo.hostName = "robonuc0.local";
	requestPort("comp0",hostInfo);

	hostInfo.publicIP = "158.49.247.3";
	hostInfo.privateIP = "192.168.1.3";
	hostInfo.hostName = "robonuc1.local";
	requestPort("comp20",hostInfo);
	
	hostInfo.publicIP = "158.49.247.4";
	hostInfo.privateIP = "192.168.1.4";
	hostInfo.hostName = "robonuc2.local";
	requestPort("comp02",hostInfo);
	
	requestPortOfComponent("comp0","robonuc0.local");
	requestPortOfComponent("comp0","robonuc1.local");
	requestPortOfComponent("comp02","robonuc3.local");
	requestIdOfComponent(10000,"robonuc0.local");
	requestIdOfComponent(10022,"robonuc1.local");
	requestIdOfComponent(10002,"robonuc3.local");
	requestHostById("comp02");
	requestHostById("comp12");
	requestHostByPort(10001);
	requestHostByPort(11111);
	requestAllCompInHost("robonuc0.local");
	requestAllCompInHost("robonuc1.local");
	requestAllCompInHost("robonuc2.local");
	requestAllCompInHost("robonuc3.local");
	requestAllComps();
 }

void SpecificWorker::requestPort(string idComp, RoboCompRCDNS::ipInfo host)
{
	try
	{
		int n = rcdns_proxy->giveMePort(idComp, host);
		if (n == 0)
		{
			printf("Fatal Error. The component already exit!\n");
		}
		else
		{
			printf("asignado para el componente: %s, el puerto %d en el host: %s\n",idComp.c_str(),n,host.hostName.c_str());
		}

	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from DNS" << e << std::endl;
	}

}

void SpecificWorker::requestRealPort()
{
	RoboCompRCDNS::ipInfo hostData;
	QList<QNetworkInterface> ifaces = QNetworkInterface::allInterfaces();
	if ( !ifaces.isEmpty() )
	{
		for(int i=0; i < ifaces.size(); i++)
		{
			unsigned int flags = ifaces[i].flags();
			bool isLoopback = (bool)(flags & QNetworkInterface::IsLoopBack);
			bool isP2P = (bool)(flags & QNetworkInterface::IsPointToPoint);
			bool isRunning = (bool)(flags & QNetworkInterface::IsRunning);

			// If this interface isn't running, we don't care about it
			if ( !isRunning ) continue;
			// We only want valid interfaces that aren't loopback/virtual and not point to point
			if ( !ifaces[i].isValid() || isLoopback || isP2P ) continue;
			QList<QHostAddress> addresses = ifaces[i].allAddresses();
			for(int a=0; a < addresses.size(); a++)
			{
				// Ignore local host
				if ( addresses[a] == QHostAddress::LocalHost ) continue;

				// Ignore non-ipv4 addresses
				if ( !addresses[a].toIPv4Address() ) continue;

				QString ip = addresses[a].toString();
				if ( ip.isEmpty() ) continue;
				printf("iface: %s          ip: %s \n",ifaces[i].name().toStdString().c_str(),ip.toStdString().c_str());
				bool ipCablePub = false;
				bool ipCablePriv = false;
				if ("10." == ip.left(3) || "127.16." == ip.left(6) || "192.168." == ip.left(8))
				{
					if ("eth" == ifaces[i].name().left(3))
					{
						hostData.privateIP = ip.toStdString();
						ipCablePub = true;
					}
					else 
					{
						if ("wlan" == ifaces[i].name().left(4) && !ipCablePub)
						{
							hostData.privateIP = ip.toStdString();
						}
						else 
						{
							if ("eth" == ifaces[i].name().left(3))
							{
								hostData.publicIP = ip.toStdString();
								ipCablePriv = true;
							}
							else 
							{
								if ("wlan" == ifaces[i].name().left(4) && !ipCablePriv)
								{
									hostData.publicIP = ip.toStdString();
								}
							}
						}
					}
				}
			}
			hostData.hostName = QHostInfo::localHostName().toStdString() + ".local";
			printf("public IP: %s, privateIP: %s, hostName: %s\n",hostData.publicIP.c_str(),hostData.privateIP.c_str(),hostData.hostName.c_str());
			if (hostData.publicIP == "" && hostData.privateIP == "" && hostData.hostName == "")
			{
				printf("ERROR! WITH YOURS NETWORK INTERFACES\n");
			}
			else
			{
				if (hostData.publicIP != "" && hostData.privateIP != "" && hostData.hostName != "")
				{
					printf("ou yeah!\n");
				}
				else
				{
					printf("WARNING! NOT CONFIGURATION COMPLETE");
				}
				
				try
				{
					int n = rcdns_proxy->giveMePort("REAL COMP", hostData);
					if (n == 0)
					{
						printf("Fatal Error. The component already exit!\n");
					}
					else
					{
						printf("asignado para el componente: %s, el puerto %d en el host: %s\n","REAL COMP",n,hostData.hostName.c_str());
					}

				}
				catch(const Ice::Exception &e)
				{
					std::cout << "Error reading from DNS" << e << std::endl;
				}
			}
		}
	}
}

void SpecificWorker::requestPortOfComponent(string idComp, string host)
{
	try
	{
		int port = rcdns_proxy->getComponentPort(idComp, host);
		if (port == 0)
		{
			printf("Error. The component not up\n");
		}
		else
		{
			printf("el puerto del componente: %s es el %d\n",idComp.c_str(),port);
		}
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from DNS" << e << std::endl;
	}

}

void SpecificWorker::requestIdOfComponent(int port, string host)
{
	try
	{
		string idComp = rcdns_proxy->getComponentId(port, host);
		if (idComp == "")
		{
			printf("Error. The component not up\n");
		}
		else
		{
			printf("el id del componente en el puerto: %d es el %s\n",port,idComp.c_str());
		}
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from DNS" << e << std::endl;
	}

}

void SpecificWorker::requestHostById(string idComp)
{
	try
	{
		string host = rcdns_proxy->getComponentHostNameById(idComp);
		if (host == "")
		{
			printf("Error. The component not up\n");
		}
		else
		{
			printf("el host del componente: %s es el %s\n",idComp.c_str(),host.c_str());
		}
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from DNS" << e << std::endl;
	}

}

void SpecificWorker::requestHostByPort(int port)
{
	try
	{
		string host = rcdns_proxy->getComponentHostNameByPort(port);
		if (host == "")
		{
			printf("Error. The component not up\n");
		}
		else
		{
			printf("el host que tiene el puerto: %d es el %s\n",port,host.c_str());
		}
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from DNS" << e << std::endl;
	}

}

void SpecificWorker::requestAllCompInHost(string host)
{
	try
	{
		RoboCompRCDNS::DnsHostsSeq dnsData;
		dnsData = rcdns_proxy->getAllCompsHost(host);
		if (dnsData.size() == 0)
		{
			printf("Error. Not hostname exist or no components up\n");
		}
		else
		{
			for(auto comp : dnsData)
			{
				printf("el host que tiene el componente %s en el puerto: %d es el %s\n",(comp.idComp).c_str(),comp.port,(comp.host.hostName).c_str());
			}
		}
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from DNS" << e << std::endl;
	}
}

void SpecificWorker::requestAllComps()
{
	try
	{
		RoboCompRCDNS::DnsHostsSeq dnsData;
		dnsData = rcdns_proxy->getAllComps();
		if (dnsData.size() == 0)
		{
			printf("Error. Not hostname exist or no components up\n");
		}
		else
		{
			for(auto comp : dnsData)
			{
				printf("host - componente - puerto: %s --- %s --- %d\n",(comp.host.hostName).c_str(),(comp.idComp).c_str(),comp.port);
			}
		}
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from DNS" << e << std::endl;
	}
}
