#include "faulhaberApi.h"

/**
* \brief Default constructor
*/
FaulHaberApi::FaulHaberApi(QString device, int baudRate) : CanBus(device, baudRate)
{
	printf("FaulHaberApi: %s %d\n", device.toStdString().c_str(), baudRate);
	//qDebug()<<"binario"<<QString::number(16,2);
}
/**
* \brief Default destructor
*/
FaulHaberApi::~FaulHaberApi()
{
	
}

void FaulHaberApi::Init_Node(int id)
{
	//tres tramas iniciales

	msg = buildMessageData(CanOpenId,id,2,0x01,0x0000,0x00,0x00000000);
	writeWaitReadMessage(&msg);
	writeWaitReadMessage(&msg);
	writeWaitReadMessage(&msg);
	writeWaitReadMessage(&msg);
	writeWaitReadMessage(&msg);
	writeWaitReadMessage(&msg);

	int result = 0;
	//switch on disable
	msg = buildMessageData(WriteObjectId,id,8,0x2B,0x6040,0x00,0x00000000);
	result = writeWaitReadMessage(&msg);

	if(id == 3)
		sleep(1);

	//ready to switch on
	msg = buildMessageData(WriteObjectId,id,8,0x2B,0x6040,0x00,0x00000006);
	result += writeWaitReadMessage(&msg);

	//switch on voltage
	msg = buildMessageData(WriteObjectId,id,8,0x2B,0x6040,0x00,0x00000007);
	result += writeWaitReadMessage(&msg);

	//operation enabled
	msg = buildMessageData(WriteObjectId,id,8,0x2B,0x6040,0x00,0x0000000F);
	result += writeWaitReadMessage(&msg);


	/*	if (result != 4)
		qFatal("error setting up node");*/

	//2B => write??
	//2F => read??

}

int FaulHaberApi::getPosition(int id)
{

	//qDebug()<<"read position"<<id;
	//msg = buildMessageData(WriteObjectId,id,8,0x40,0x0000,0x00,0x00000000);
    msg = buildMessageData(0x300,id,8,0x40,0x0000,0x00,0x00000000);
	int a=writeWaitReadMessage(&msg);
	
	if (id == 1)
		printf("output  %lx, %x, %x, %x, %x, %x, %x, %x, %x, %x %d\n ", msg.Id, msg.Size, msg.Data[0], msg.Data[1], msg.Data[2], msg.Data[3], msg.Data[4], msg.Data[5],	 msg.Data[6], msg.Data[7], msg.Timestamp);
	
	if(a == 1)
		return readIntegerResponse(msg);
	else
	{
		printf("output  %lx, %x, %x, %x, %x, %x, %x, %x, %x, %x %d\n ", msg.Id, msg.Size, msg.Data[0], msg.Data[1], msg.Data[2], msg.Data[3], msg.Data[4], msg.Data[5],	 msg.Data[6], msg.Data[7], msg.Timestamp);
		return -1;
	}
}

int FaulHaberApi::getPositionExternalEncoder(int id)
{

	//qDebug()<<"read position external encoder"<<id;
	msg = buildMessageData(0x300,id,8,0xb2,0x0005,0x00,0x00000000);
	int a=writeWaitReadMessage(&msg);
	//printf("output  %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x \n ", msg.Id,msg.Size, msg.Data[0], msg.Data[1], msg.Data[2], msg.Data[3], msg.Data[4], msg.Data[5],	 msg.Data[6], msg.Data[7], msg.Timestamp );
	
	if(a == 1)
		return readIntegerResponse(msg);
	else
		return -1;
	
}

//sets value of motor internal encoder to value
void FaulHaberApi::setInternalEncoderValue(int id, int value)
{
    unsigned char D3,D2,D1,D0;

    D0 = (value & 0xFF000000 )/0X1000000;
    D1 = (value & 0xFF0000 )/0X10000;
    D2 = (value & 0xFF00 )/0X100;
    D3 = (value & 0xFF );
	
   // Envio_Trama(Handle, (0x300+ID),5,0xB8,D3,D2,D1,D0,0x00,0x00,0x00); // LOAD ABSOLUTE
	
	msg = buildRawMessageData(0x300+id,5,0xB8,D3,D2,D1,D0,0x00,0x00,0x00);
	writeWaitReadMessage(&msg);
}

int FaulHaberApi::syncGetPosition(int node_count,int* nodeIds,int* positions)
{

	for(int count=0;count<node_count;count++)
	{
		//msgs[count] = buildMessageData(WriteObjectId,nodeIds[count],8,0x40,0x0000,0x00,0x0000000000);
		msgs[count] = buildMessageData(0x300,nodeIds[count],8,0x40,0x0000,0x00,0x0000000000);
	}
	int status = multiWriteWaitReadMessage(msgs, node_count);
	int pos = 0;
	for(int count=0;count<node_count;count++)
	{
		pos = 0;
		while ((int64_t)nodeIds[pos] != (int64_t)msgs[count].Id %0x10)
		{
			pos++;
			if(pos > node_count +1)
				return -1;
		}
		positions[pos] = readIntegerResponse(msgs[count]);
	}
	
	
	return status;
}

void FaulHaberApi::setPosition(int id, int position)
{
	qDebug()<<"set position"<<id<<position;
	msg = buildFaulhaberCommand(id,0xB4,position);//set position
	int status = writeWaitReadMessage(&msg);
	
	msg = buildFaulhaberCommand(id,0x3C, 0);//initiate motion
	status += writeWaitReadMessage(&msg);
	qDebug()<<"set position result"<<status;
}

int FaulHaberApi::syncSetPosition(int node_count,int* nodeIds,int* positions)
{
	for(int count=0;count<node_count;count++)
	{
		qDebug()<<"sync"<<nodeIds[count]<<positions[count];
		msgs[count] = buildFaulhaberCommand(nodeIds[count],0xB4,positions[count]);//set position
	}
	int status = multiWriteWaitReadMessage(msgs, node_count);
	
	for(int count=0;count<node_count;count++)
	{
		msgs[count] = buildFaulhaberCommand(nodeIds[count],0x3C,positions[count]);//initiate motion
	}
	status += multiWriteWaitReadMessage(msgs, node_count);
	
	return status;
}


void FaulHaberApi::goHome(int id)
{
	msg = buildFaulhaberCommand(id,0x2F,0); //go end track
	/*int status = */writeWaitReadMessage(&msg);
//	qDebug()<<"go home result"<<status;
}


void FaulHaberApi::setZero(int id)
{
	msg = buildFaulhaberCommand(id,0xB8,0);//set position
	/*int status =*/ writeWaitReadMessage(&msg);
//	qDebug()<<"set home position result "<<status;
}



void FaulHaberApi::setAceleration(int id, int aceleration)
{
	msg = buildMessageData(WriteObjectId,id,8,0x23,0x6083,0x00,aceleration);	
	/*int status =*/ writeWaitReadMessage(&msg);
//	qDebug()<<"set aceleration"<<status;
}
void FaulHaberApi::setDeceleration(int id, int deceleration)
{
	msg = buildMessageData(WriteObjectId,id,8,0x23,0x6084,0x00,deceleration);	
	/*int status = */writeWaitReadMessage(&msg);
//	qDebug()<<"set deceleration"<<status;
}

int FaulHaberApi::setVelocity(int id, int vel)
{
	msg = buildMessageData(WriteObjectId,id,8,0x23,0x6084,0x00,vel);
	int status = writeWaitReadMessage(&msg);
//qDebug()<<"set velocity"<<status;
	return status;
}
int FaulHaberApi::syncSetVelocity(int node_count,int* nodeIds,int* velocities)
{
	for(int count=0;count<node_count;count++)
	{
		msgs[count] = buildMessageData(WriteObjectId,nodeIds[count],8,0x23,0x6084,0x00,velocities[count]);//set position
	}
	int status = multiWriteWaitReadMessage(msgs, node_count);
	return status;
}



void FaulHaberApi::enableControlPositionMode(int id)
{
	msg = buildMessageData(WriteObjectId,id,8,0x2F,0x6060,0x00,0x00000001);
	/*int status =*/ writeWaitReadMessage(&msg);
//	qDebug()<<"enable mode result:"<<status;
}

void FaulHaberApi::enableCommandMode(int id)
{
	msg = buildMessageData(WriteObjectId,id,8,0x2F,0x6060,0x00,0x000000FF);
	/*int status = */writeWaitReadMessage(&msg);
//	qDebug()<<"enable mode result "<<status;
}



int FaulHaberApi::enablePower(int id)
{
	msg = buildMessageData(WriteObjectId,id,8,0x2B,0x3004, 0x00, 0x00000001);
	int status = writeWaitReadMessage(&msg);
	return status;
}
int FaulHaberApi::disablePower(int id)
{
	msg = buildMessageData(WriteObjectId,id,8,0x2B,0x3004, 0x00, 0x00000001);//no bien construida
	int status = writeWaitReadMessage(&msg);
	return status;
}

VSCAN_MSG FaulHaberApi::buildFaulhaberCommand(uint8_t nodeId,uint8_t CommandSpecifier, uint32_t obj_data)
{
	VSCAN_MSG msg;
	msg.Flags = VSCAN_FLAGS_STANDARD;
	msg.Id = FaulhaberCommandId;
	msg.Id |= nodeId;
	msg.Size = 5;
	msg.Data[0] = CommandSpecifier;
	msg.Data[1] = (obj_data&0x000000FF);
	msg.Data[2] = (obj_data&0x0000FF00)>>8;
	msg.Data[3] = (obj_data&0x00FF0000)>>16;
	msg.Data[4] = (obj_data&0xFF000000)>>24;
	msg.Data[5] = 0x00;
	msg.Data[6] = 0x00;
	msg.Data[7] = 0x00;
	return msg;

}
#define pasos_mm 166.66
#define BRAZO 27  // 27 milimetros entre el centro ocular y el brazo de mando del ojo

//deprecated please use units2steps function instead
int FaulHaberApi::convertir_Radianes_Pasos(float angulo, bool inverted)
{
	if(inverted != true)
		angulo = -angulo;
	
	return (tan(angulo)*BRAZO*pasos_mm);   // Convertido a radianes
}

//deprecated please use steps2units function instead
float FaulHaberApi::convertir_Pasos_Radianes(int pasos, bool inverted)
{
	float aux =(atan2 (pasos,(pasos_mm*BRAZO)));
	if(inverted != true)
		aux = -aux;
	return aux;
}

float FaulHaberApi::steps2units(float steps, float step_range, float unit_range, float offset)
{
		//cout<<" steps2units: "<<steps<<" "<<step_range<<" "<<unit_range<<" "<<offset<<endl;
        //return ((float)(p-steps_value_at_zero_units)) * (unit_range/step_range);
		//return (float)(steps-offset) * (unit_range/(step_range-offset));
		return (float) ( (steps - offset) * (unit_range / step_range) );
	
}

int FaulHaberApi::units2steps(float units, float step_range, float unit_range, float offset)
{
	 //cout<<" units2steps: "<<units<<" "<<step_range<<" "<<unit_range<<" "<<offset<<endl;
	 //return (float)((units * (step_range - offset) / unit_range) + offset);
	 return (float) (((units * step_range) / unit_range) + offset);  
}


void FaulHaberApi::resetBus()
{
	VSCAN_MSG msg = buildMessageData(0x00,0x00,2,0x01,0x0000,0x00,0x00000000);
	writeWaitReadMessage(&msg);
	writeWaitReadMessage(&msg);
	writeWaitReadMessage(&msg);
	sleep(5);
}

