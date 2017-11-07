#include "canbus.h"


/**
* \brief Default constructor
*/
CanBus::CanBus(QString device, int baudRate)
{
	printf("CanBus::CanBus:  %s %d\n", device.toStdString().c_str(), baudRate);
	rDebug("CanBus initializing");
	// Open and initialize the device
	devHandler = VSCAN_Open(device.toAscii().data(),  VSCAN_MODE_NORMAL);
	if(devHandler < 0)
	{
		qFatal("Failed to open Can device ");
		return;
	}
	rDebug("Can device "+device+" oppened correctly");
	
	if(setBaudRate(baudRate) != 0)
		rDebug("Failed setting baudRate");
}
/**
* \brief Default destructor
*/
CanBus::~CanBus()
{
	
}
/// PRIVATE METHODS
int CanBus::setBaudRate(int baudRate)
{
	void * speed;
	switch(baudRate)
	{
		case 20000:	speed=VSCAN_SPEED_20K;
					break;
		case 50000:	speed=VSCAN_SPEED_50K;
					break;
		case 100000:speed=VSCAN_SPEED_100K;
					break;
		case 125000:speed=VSCAN_SPEED_125K;
					break;
		case 250000:speed=VSCAN_SPEED_250K;
					break;
		case 500000:speed=VSCAN_SPEED_500K;
					break;
		case 800000:speed=VSCAN_SPEED_800K;
					break;
		case 1000000:speed=VSCAN_SPEED_1M;
					break;
		default:
			rDebug("ERROR: "+QString::number(baudRate)+" no es un valor de baudRate válido (20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000)");
			return -1;
	}
	char string[33];
	int status = VSCAN_Ioctl(devHandler, VSCAN_IOCTL_SET_SPEED, VSCAN_SPEED_1M);
	VSCAN_GetErrorString(status,string,32);
	 printf(" Apertura puerto %s\n",string);
	 
	status = VSCAN_Ioctl(devHandler, VSCAN_IOCTL_SET_TIMESTAMP, VSCAN_TIMESTAMP_ON);
    VSCAN_GetErrorString(status,string,32);
    printf(" SET TIME STAMP %s\n",string);
	return status;
}

int CanBus::writeWaitReadMessage(VSCAN_MSG* msg)
{
	DWORD written,read;
	VSCAN_MSG sended;
	int retries=0, right_response = 0;
//	printf("------------------------------------\n");
//	printf("Write messages\n");
//	printMessageData(*msg);

    int status = VSCAN_Write(devHandler,msg,1,&written);
	VSCAN_Flush(devHandler);
	if(status != 0)
	{
		printf("writeWaitReadMessage() ERROR: El comando no se escribio correctamente\n");
		return -1;
	}
//	if(msg->Id == 0x0)
//		return 1;
	
	usleep(2000);
	
	memcpy(&sended, msg, sizeof(VSCAN_MSG));
//	printf("received\n");
	do{
		
		if(VSCAN_Read(devHandler, msg, 1, &read) == 0 and read != 0)
		{
//			printMessageData(*msg);
			right_response = checkMessage(&sended, msg);
		}
		usleep(2000);
		retries++;
	}while(right_response != 1 and retries < MAX_READ_RETRIES);
	
	if(retries >= MAX_READ_RETRIES)
	{
		printf("writeWaitReadMessage() ERROR: No se pudo leer la respuesta al comando enviado (retries = %d)\n", retries);
		printMessageData(sended);
		return -1;
	}
	else
	{
// 		qDebug()<<"tryed: "<<retries<<endl;
// 		qDebug()<<"good response";
		return 1;
	}
}

//leer la respuesta mediante los ids no por el contador del msg_count

int CanBus::multiWriteWaitReadMessage(VSCAN_MSG* msgs, int msg_count)
{
	int status, retries=0;
	uint8_t rec_count = 0, count_readed = 0;
	DWORD   written,read;
	VSCAN_MSG sended_msgs[msg_count] , readed_msgs[msg_count];
	memcpy(sended_msgs, msgs, sizeof(VSCAN_MSG)*msg_count);
		
	QVector<VSCAN_MSG> cmds_tocheck;
	for(int x=0;x<msg_count;x++)
		cmds_tocheck.push_back(msgs[x] );

//	printf("------------------------------------\n");
//	printf("Initial messages:\n");
//	for(mcount=0;mcount<msg_count;mcount++)
//		printMessageData(msgs[mcount]);
	status = VSCAN_Write(devHandler,msgs,msg_count,&written);
	VSCAN_Flush(devHandler);
	if(msgs[0].Id / 0x100 == 3)
		return 1;
	
	if(status != 0)
	{
		printf("writeWaitReadMessage() ERROR: El comando no se escribio correctamente\n");
		return -1;
	}
    
	do{
		if(VSCAN_Read(devHandler, readed_msgs, msg_count, &read) == 0 and read != 0)
		{
			for( int i=0;i< msg_count;i++)
			{
				QVector<VSCAN_MSG>::iterator it;
				for(int j = 0;j < cmds_tocheck.size();j++)		//sended messages
				{
					if(checkMessage(&cmds_tocheck[j],&readed_msgs[i]) == 1)
					{
						memcpy(&msgs[i], &readed_msgs[i], sizeof(VSCAN_MSG));
						cmds_tocheck.remove(j);
						break;
					}
				}
			}
		}
		usleep(20000);
		retries++;
	}while(cmds_tocheck.size() > 0 and retries < MAX_READ_RETRIES);
	if(retries >= MAX_READ_RETRIES)
	{
		printf("writeWaitReadMessage() ERROR: No se pudo leer la respuesta al comando enviado (retries = %d)\n", retries);
		while(cmds_tocheck.size() > 0)
		{
			printMessageData(cmds_tocheck.front());
			cmds_tocheck.pop_front();
		}
		return -1;
	}
	else
	{
//		qDebug()<<"good response";
		return 1;
	}
}

//return 1 ==> correct, 0 ==> incorrect, 2 => unknown type
int CanBus::checkMessage(VSCAN_MSG *send, VSCAN_MSG *received)
{
	int correct = 0;
	if (send->Id / 0x100 == 6)
	{
		if(received->Id / 0x100 == 5 and send->Id %0x10 == received->Id %0x10 and send->Data[2] == received->Data[2])  
			correct = 1;
	}
	else if (send->Id / 0x100 == 3 and send->Data[0] == received->Data[0])
		correct = 1;

	return correct;
}

//Check mensajes: 
//enviado  => printMessageData() Id: 0x601, Size: 0X8 | Cmd Type: 0x2b, Obj. Id: 0x40 0x60, SubId: 00, Data: 0x6  00 00 00
//recivido => printMessageData() Id: 0x581, Size: 0X8 | Cmd Type: 0x60, Obj. Id: 0x40 0x60, SubId: 00, Data: 00  00 00 00


void CanBus::printMessageData(VSCAN_MSG msg)
{
	if(msg.Size == 5)
	{
		printf("printMessageData() Id: %#02lx, Size: %d | Cmd: %#02x, Data: %#02x %#02x %#02x %#02x \n",
		msg.Id,
		msg.Size,
		msg.Data[0],
		msg.Data[1],
		msg.Data[2],
		msg.Data[3],
		msg.Data[4]);
	}
	else
	{
		printf("printMessageData() Id: %#02lx, Size: %#02X | Cmd Type: %#02x, Obj. Id: %#02x %#02x, SubId: %#02x, Data: %#02x  %#02x %#02x %#02x \n",
		msg.Id,
		msg.Size,
		msg.Data[0],
		msg.Data[1],
		msg.Data[2],
		msg.Data[3],
		msg.Data[4],
		msg.Data[5],
		msg.Data[6],
		msg.Data[7] );
	}
}


VSCAN_MSG CanBus::buildMessageData(uint16_t type, uint8_t nodeId,uint8_t size,uint8_t CommandSpecifier, uint16_t obj_id,uint8_t obj_subid, uint32_t obj_data)
{
	VSCAN_MSG msg;
	msg.Flags = VSCAN_FLAGS_STANDARD;
	msg.Id = type;
	if(type != 0)
		msg.Id |= nodeId;
	msg.Size = size;
	msg.Data[0] = CommandSpecifier;
	msg.Data[1] = obj_id&0x00FF;
	msg.Data[2] = (obj_id&0xFF00)>>8;
	msg.Data[3] = obj_subid;
	msg.Data[4] = (obj_data&0x000000FF);
	msg.Data[5] = (obj_data&0x0000FF00)>>8;
	msg.Data[6] = (obj_data&0x00FF0000)>>16;
	msg.Data[7] = (obj_data&0xFF000000)>>24;
	msg.Timestamp = 1000;
	
	return msg;
}

VSCAN_MSG CanBus::buildRawMessageData(uint32_t ID, uint8_t Size, uint8_t D0, uint8_t D1, uint8_t D2, uint8_t D3, uint8_t D4, uint8_t D5, uint8_t D6, uint8_t D7)
{
	VSCAN_MSG msg;
	
	msg.Flags = VSCAN_FLAGS_STANDARD;
    msg.Id = ID;
    msg.Size = Size;
    msg.Data[0] = D0;
    msg.Data[1] = D1;
    msg.Data[2] = D2;
    msg.Data[3] = D3;
    msg.Data[4] = D4;
    msg.Data[5] = D5;
    msg.Data[6] = D6;
    msg.Data[7] = D7;
    msg.Timestamp = 1000;

	//printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u \n", msg.Id, msg.Size, msg.Data[0], msg.Data[1], msg.Data[2], msg.Data[3], msg.Data[4], msg.Data[5], msg.Data[6], msg.Data[7],msg.Timestamp);
	
	return msg;
}

//transforma la devolucion de canbus en un entero => lector posicion, velocidad...
int CanBus::readIntegerResponse(VSCAN_MSG msg)
{
	int result = 0;
	//old deprecated way, it seems that it was never used, left here just in case
// 	result += msg.Data[7]<<24;
// 	result += msg.Data[6]<<16;
// 	result += msg.Data[5]<<8;
// 	result += msg.Data[4];

	result =  (msg.Data[4])*0x1000000;
	result += (msg.Data[3])*0x10000;
	result += (msg.Data[2])*0x100;
	result += msg.Data[1];
	
	
	return result;
}

