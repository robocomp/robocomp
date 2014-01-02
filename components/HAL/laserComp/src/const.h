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
#ifndef CONST_H
#define CONST_H

#define PROGRAM_NAME    "RoboComp::Laser"


#define LASER_DRIVER_PROPERTY_NAME        "Laser.Driver"
#define LASER_DRIVER_PROPERTY_DEFAULT     "HokuyoURG"
#define LASER_DEVICE_PROPERTY_NAME        "Laser.Device"
#define LASER_DEVICE_PROPERTY_DEFAULT     "/dev/ttyACM0"
#define LASER_START_PROPERTY_NAME         "Laser.StartValue"
#define LASER_START_PROPERTY_DEFAULT      44
#define LASER_END_PROPERTY_NAME           "Laser.EndValue"
#define LASER_END_PROPERTY_DEFAULT        725
#define LASER_SKIP_PROPERTY_NAME          "Laser.SkipValue"
#define LASER_SKIP_PROPERTY_DEFAULT       1
#define LASER_SAMPLERATE_PROPERTY_NAME    "Laser.SampleRate"
#define LASER_SAMPLERATE_PROPERTY_DEFAULT 100

#define LASER_RESOLUTION_PROPERTY_NAME    "Laser.angleRes"
#define LASER_ANGLE_RESOLUTION_DEFAULT    0.3515625*M_PIl/180.
#define LASER_INITIAL_ANGLE_PROPERTY_NAME "Laser.angleIni"
#define LASER_INITIAL_ANGLE_DEFAULT 	  -135*M_PI/180

#define LASER_MAX_RANGE_PROPERTY_NAME     "Laser.maxRange"
#define LASER_MAX_RANGE_DEFAULT 	  4094
#define LASER_MIN_RANGE_PROPERTY_NAME     "Laser.minRange"
#define LASER_MIN_RANGE_DEFAULT 	  40
#define LASER_STEP_MIN_ANGLE_DEFAULT 	  44


// Laser poweroff, in mseconds
#define LASER_TIMEOUT   10000

// Laser commands sizes
#define LASER_CMD_GET_DATA_SZ   10
#define LASER_CMD_POWER_SZ      3

// Laser commands
#define LASER_CMD_POWER_ON  "L1\n"
#define LASER_CMD_POWER_OFF "L0\n"

#endif
