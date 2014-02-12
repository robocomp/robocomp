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
#ifndef FAULHABERAPI_H
#define FAULHABERAPI_H

#include "canbus.h"

#define CanOpenId 0x000
#define StatusWordId  0x180
#define ControlWordId 0x200
#define FaulhaberDataId 0x280
#define FaulhaberCommandId 0x300
#define ReadObjectId 0x580
#define WriteObjectId 0x600

#define MAX_MOTORS 8

class FaulHaberApi : public CanBus
{
  public:
	FaulHaberApi(QString device, int baudRate);
	~FaulHaberApi();
	void Init_Node(int id);
 
	//Position
	int getPosition(int id);
	int getPositionExternalEncoder(int id);
	int syncGetPosition(int node_count,int* nodeIds,int* positions);//probar
	void setPosition(int id, int position);
	int syncSetPosition(int node_count,int* nodeIds,int* positions);//probar
	
	
	void setZero(int id);
	
	void goHome(int id);
	
	void setInternalEncoderValue(int id, int value);
	
	//Speed
	int setVelocity(int id, int vel);//probar
	int syncSetVelocity(int node_count,int* nodeIds,int* velocities);//probar
	
	
	//Aceleration
	void setAceleration(int id, int aceleration);//probar
	void setDeceleration(int id, int deceleration);//probar
	
	//Commands mode
	void enableCommandMode(int id);
	void enableControlPositionMode(int id);
	
	
	
	void resetBus();
	
	int enablePower(int id); //probar
	int disablePower(int id);
	
	int convertir_Radianes_Pasos(float angulo,bool inverted);
	float convertir_Pasos_Radianes(int pasos,bool inverted);

	float steps2units(float steps,float step_range, float unit_range, float offset);
	int units2steps(float units, float step_range, float unit_range, float offset);
	
	//Faulhaber Specific command
	VSCAN_MSG buildFaulhaberCommand(uint8_t nodeId,uint8_t CommandSpecifier, uint32_t obj_data);

  private:
	VSCAN_MSG msg,msgs[MAX_MOTORS];

	
	
};

#endif

