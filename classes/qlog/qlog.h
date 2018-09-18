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
#ifndef QLOG_H
#define QLOG_H

#include "config.h"
#include <QtCore>
#include <iostream>

#if COMPILE_LOGGERCOMP==1
	#include <Logger.h>
#endif

#ifndef Q_MOC_RUN
	#include <boost/format.hpp> 

using namespace std;



#include <boost/format.hpp>
#endif


#define SetLoggerInstance(x) loggerInstance = x
#define rDebug2(strng) qLog::getInstance()->send(std::string(__FILE__),__LINE__,std::string(__func__),boost::str(boost::format strng ),std::string("Debug"))
#define rDebug(strng) qLog::getInstance()->send(std::string(__FILE__),__LINE__,std::string(__func__),strng,std::string("Debug"))
#define rInfo(strng) qLog::getInstance()->send(std::string(__FILE__),__LINE__,std::string(__func__),strng,std::string("Info"))
#define rError(strng) qLog::getInstance()->send(std::string(__FILE__),__LINE__,std::string(__func__),strng,std::string("Error"))

class qLog
{
private:
#if COMPILE_LOGGERCOMP==1
	RoboCompLogger::LogMessage mess;
	RoboCompLogger::LoggerPrx prx;
public:
	void setProxy(std::string endpoint,RoboCompLogger::LoggerPrx _prx);
	void sendLogger();
#endif
private:
	 std::string file;
	 std::string func;
	 int nLine;
	 std::string type;
	 std::string timeStamp;
	 std::string fullpath;
	 std::string message;
	 std::string sender;
	 std::string method;
	 
	QString log;
	static qLog *logger;
	void showConsole();
  public:
	~qLog();
	qLog();
	static qLog* getInstance();
	void send(std::string file, int line,std::string func, std::string strng,std::string type);
	void send(std::string file, int line,std::string func, const char* strng,std::string type);
	void send(std::string file, int line,std::string func, QString strng,std::string type);
};

#endif
