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
#include "genericmonitor.h"
/**
* \brief Default constructor
*/
GenericMonitor::GenericMonitor(GenericWorker *_worker,Ice::CommunicatorPtr _communicator)
{
	worker = _worker;
	this->communicator = _communicator;
	period = 100;
	state = RoboCompCommonBehavior::State::Starting;
	QObject::connect(this, SIGNAL(initializeWorker(int)), worker, SLOT(initialize(int)));
}
/**
* \brief Default destructor
*/
GenericMonitor::~GenericMonitor()
{

}

/**
* \brief Get component execution state
* @return State Component state
*/
RoboCompCommonBehavior::State GenericMonitor::getState()
{
	return state;
}

/**
* \brief Get worker period
* @return int Worker period in ms
*/
int GenericMonitor::getPeriod()
{
	return period;
}
/**
* \brief Change worker period
* @param per Period in ms
*/
void GenericMonitor::setPeriod(int _period)
{
	period =_period;
	worker->setPeriod(_period);
}
/**
* \brief Kill component
*/
void GenericMonitor::killYourSelf()
{
	rDebug("Killing myself");
	worker->killYourSelf();
	emit kill();

}
/**
* \brief Get Component time awake
* @return int Time alive in seconds
*/
int GenericMonitor::timeAwake()
{
	return initialTime.secsTo(QTime::currentTime());
}
/**
* \brief Return components parameters
* @return  AttrList Configuration parameters list
*/
RoboCompCommonBehavior::ParameterList GenericMonitor::getParameterList()
{
	return config_params;
}
/**
* \brief Change configurations parameters to worker
* @param l Configuration parameters list
*/
void GenericMonitor::setParameterList(RoboCompCommonBehavior::ParameterList l)
{
	rInfo("Changing configuration params");
	sendParamsToWorker(l);
}

/**
* \brief Read parameters from pconf file. This method will be empty if there is not any pconf file available.
* @param l Configuration parameters list
*/
void GenericMonitor::readPConfParams(RoboCompCommonBehavior::ParameterList &params)
{
	//nothing to do
}

//Ice method to read a variable from file
//name, parameter config value
//return value of parameter config
//default value for the parameter
//return false if the parameter does not exist. Throw exception in other case.
//if you need one parameter mandatory you can pass empty string in default_value
bool GenericMonitor::configGetString(const std::string prefix, const std::string name, std::string &value, const std::string default_value, QStringList *list)
{
	return GenericMonitor::configGetString(communicator, prefix, name, value, default_value, list);
}

bool GenericMonitor::configGetString(Ice::CommunicatorPtr communicator, const std::string prefix, const std::string name, std::string &value, const std::string default_value, QStringList *list)
{
	std::string compound = name;
	if (prefix.size() > 0) compound = prefix+std::string(".")+name;

	value = communicator->getProperties()->getProperty(compound);

	if ( value.length() == 0)
	{
		if (default_value.length() != 0)
		{
			value = default_value;
			return false;
		}
		else if (default_value.length() == 0)
		{
			QString error = QString("Error: can't get configuration string for variable without default value: ")+QString::fromStdString(compound);
			qDebug() << error;
			throw error;
		}
	}

	if (list != NULL)
	{
		if (list->contains(QString::fromStdString(value)) == false)
		{
			qFatal("Reading config file: %s is not a valid string", compound.c_str());
			rError("Reading config file:"+compound+" is not a valid string");
		}
		QString error = QString("not valid configuration value");
		qDebug() << error;
		throw error;
	}

	auto parts = QString::fromStdString(value).split("@");
	QString variableName=QString::fromStdString(compound);


	if (parts.size() > 1)
	{
		if (parts[0].size() > 0)
		{
			variableName = parts[0];
		}
		parts.removeFirst();
		value = std::string("@") + parts.join("@").toStdString();
	}

// 	printf("variableName = %s\n", variableName.toStdString().c_str());
// 	printf("value = %s\n", value.c_str());


	if (value[0]=='@')
	{
		QString qstr = QString::fromStdString(value).remove(0,1);
		QFile ff(qstr);
		if (not ff.exists())
		{
			qFatal("Not such file: %s\n", qstr.toStdString().c_str());
		}
		if (!ff.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			qFatal("Can't open file: %s\n", qstr.toStdString().c_str());
		}

		bool found = false;
		while (!ff.atEnd())
		{
			QString content = QString(ff.readLine()).simplified();
// 			printf("line: %s\n", content.toStdString().c_str());

			if (content.startsWith(variableName))
			{
// 				printf("swn %s\n", content.toStdString().c_str());
				content = content.right(content.size()-variableName.size()).simplified();
// 				printf("swn %s\n", content.toStdString().c_str());
				if (content.startsWith("="))
				{
					content = content.remove(0,1).simplified();
					value = content.toStdString();
					found = true;
				}
				else
				{
					printf("warning (=) %s\n", content.toStdString().c_str());
				}

			}
		}
		if (not found)
		{
		}
	}
	std::cout << compound << " " << value << std::endl;
	return true;
}

