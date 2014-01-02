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
#include <Ice/Ice.h>
#include <Ice/Application.h>
#include <IceUtil/IceUtil.h>
#include <QtCore>

using namespace std;


namespace RoboComp
{

	class Application : public Ice::Application
	{

	private:
		virtual void initialize() = 0;

	public:
		virtual int run(int, char*[]) = 0;
		// Helper functions for read properties from config file
		bool configGetString( const std::string name, std::string &value,  const std::string default_value = "" , QStringList *list = NULL);
		bool configGetInt( const std::string name, int &value, const int default_value  = 0 , QList< int > *list = NULL );
		bool configGetBool (const std::string name, bool &value, const int default_value  = -1 );
		bool configGetFloat( const std::string name, float &value, const float default_value = 0, QList< int > *list = NULL);
		std::string getProxyString(const std::string name);
	};

}
