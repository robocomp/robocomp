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
#ifndef CANBUS_H
#define CANBUS_H

#include <vs_can_api.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include <QtCore>

#include <qlog/qlog.h>

//DEVICE MODES
#define MODE_OFF 0x00000000
#define MODE_CURRENT 0x00000002
#define MODE_VEL 0x00000003
#define MODE_SUBVEL 0x00000005
#define MODE_POS 0x00000007

#define MAX_READ_RETRIES 20


class CanBus
{
  public:
	CanBus(QString device,int baudRate);
	~CanBus();

	int setBaudRate(int baudRate);
	int writeWaitReadMessage(VSCAN_MSG* msg);
	int multiWriteWaitReadMessage(VSCAN_MSG* msgs, int msg_count);
	
	void printMessageData(VSCAN_MSG msg);
	
	VSCAN_MSG buildMessageData(uint16_t type, uint8_t nodeId,uint8_t size,uint8_t CommandSpecifier, uint16_t obj_id,uint8_t obj_subid, uint32_t obj_data);
	VSCAN_MSG buildRawMessageData(uint32_t ID, uint8_t Size, uint8_t D0, uint8_t D1, uint8_t D2, uint8_t D3, uint8_t D4, uint8_t D5, uint8_t D6, uint8_t D7);
	
	int readIntegerResponse(VSCAN_MSG msg);
	
	int checkMessage(VSCAN_MSG* send, VSCAN_MSG* received);
	
//   private:
	int devHandler;
	

	
	
};


#endif

