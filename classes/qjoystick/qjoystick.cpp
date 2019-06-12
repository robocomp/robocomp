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
#include "qjoystick.h"

QJoyStick::QJoyStick(QString device, int32_t axes, QObject *parent) : QThread(parent)
{
    deviceName =  device ;

    info.axes = axes;
    info.buttons = 2;
    info.version = 0x000800;
    
    data_sz = sizeof(js_event);
}


QJoyStick::~QJoyStick()
{
  close(fd);
}

bool QJoyStick::openQJoy()
{
	qWarning( "[qjoystick]: Connecting to device: %s", deviceName.toLatin1().data() );

	if ((fd = open(deviceName.toLatin1().data() , O_RDONLY))<0)
	{
		qWarning( "[qjoystick]: Failed opening device." );
		return false;
	}


	ioctl(fd, JSIOCGVERSION, &(info.version));
	ioctl(fd, JSIOCGAXES, &(info.axes));
	ioctl(fd, JSIOCGBUTTONS, &(info.buttons));
	ioctl(fd, JSIOCGNAME(JOYSTICK_VERSION_NAME_LENGTH), info.name);


	qWarning("[qjoystick]: Device opened: name [%s], version [%8X], axes [%2d], buttons [%2d]", info.name, info.version, info.axes, info.buttons );
	return true;
}

bool QJoyStick::cmpJoyEv( js_event src, js_event dst )
{
    return (src.value == dst.value) && (src.type == dst.type) && (src.number == dst.number);
}

void QJoyStick::run( )
{
	for (;;)
	{
		if (read(fd, &data, data_sz) == data_sz)
		{
			emit (inputEvent(data.value, data.type, data.number));
		}
		usleep(1);
	}
}

void QJoyStick::stop()
{
    quit();
}
