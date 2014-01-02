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
#include "rapplication.h"

namespace RoboComp
{
	bool Application::configGetString( const std::string name, std::string &value,  const std::string default_value, QStringList *list)
	{
		value = communicator()->getProperties()->getProperty( name );
		if ( value.length() == 0)
		{
			std::cout << name << " property does not exist. Using default value." << std::endl;
			value = default_value;
			return false;
		}
		if(list != NULL)
		{
			if (list->contains(QString::fromStdString(value)) == false)
			{
				qFatal("Reading config file: %s is not a valid string", name.c_str());
			}
		}
		std::cout << name << " " << value << std::endl;
		return true;
	}

	bool Application::configGetInt( const std::string name, int &value, const int default_value, QList< int > *list )
	{
		string tmp;
	
		tmp = communicator()->getProperties()->getProperty( name );
		if ( tmp.length() == 0)
		{
			std::cout << name << " property does not exist. Using default value." << std::endl;
			value = default_value;
			return false;
		}
		value = std::atoi( tmp.c_str() );
		if(list != NULL)
		{
			if (list->contains(value) == false)
			{
				qFatal("Reading config file: %s is not a valid integer", name.c_str());
			}
		}
		std::cout << name << " " << value << std::endl;
		return true;
	}

	bool Application::configGetBool (const std::string name, bool &value, const int default_value)
	{
		string tmp;
		tmp = communicator()->getProperties()->getProperty( name );
		if ( tmp.length() == 0 )
		{
			if (default_value != -1)
			{
				std::cout << name << " property does not exist. Using default value." << std::endl;
				value = default_value;
				return false;
			}
			else
			{
				printf("Can't read configuration parameter %s. Got %s.", name.c_str(), tmp.c_str());
				qFatal("Error\n");
			}
		}
		if(tmp == "true" or tmp == "True" or tmp == "yes" or tmp == "Yes" or tmp == "1")
		{
			value = true;
		} 
		else if (tmp == "false" or tmp == "False" or tmp == "no" or tmp == "No" or tmp == "0")
		{
			value = false;
		}
		else
		{
			qFatal("Reading config file: '%s' is not a valid boolean", name.c_str());
		}
		std::cout << name << " " << value << std::endl;
		return true;
	}

	bool Application::configGetFloat( const std::string name, float &value, const float default_value, QList< int > *list )
	{
		QString tmp;
	
		tmp = QString::fromStdString(communicator()->getProperties()->getProperty( name ));
		if ( tmp.length() == 0)
		{
			std::cout << name << " property does not exist. Using default value." << std::endl;
			value = default_value;
			return false;
		}
		value = tmp.toFloat();
		if(list != NULL)
		{
			if (list->contains(value) == false)
			{
				qFatal("Reading config file: '%s' is not a valid float", name.c_str());
			}
		}
		std::cout << name << " " << value << std::endl;
		return true;
	}
	std::string Application::getProxyString(const std::string name)
	{
		std::string proxy;
		proxy = communicator()->getProperties()->getProperty( name );
		cout << "[" << __FILE__ << "]: Loading [" << proxy << "] proxy at '" << name << "'..." << endl;
		//rInfo("Application::Loading "+QString::fromStdString(proxy)+" at "+QString::fromStdString(name));
		if( proxy.empty() )
		{
			cout << "[" << __FILE__ << "]: Error loading proxy config!" << endl;
			return "";
		}
		else
		  return proxy;
	}
}

