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
#ifndef Q4SERIALPORT_H
#define Q4SERIALPORT_H

#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <termios.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <QIODevice>
#include <QtCore>

/**
 * @author Pablo Márquez Neila, Wayne Roth, Ricardo Royo, Pablo Bustos, lots of people.
 * 
 * No es portable. Sólo para linux.
 *
 * */

extern int errno;


class Lector: public QThread
{

public:
	Lector(QFile *_f){f=_f;};
	~Lector(){};
	QFile *f;
	void run()
	{
		for(;;)
		{
			f->waitForReadyRead(-1);
		}
	};
};


class QSerialPort :  public QIODevice
{
Q_OBJECT
public:
	typedef enum _FlowType {
	    FLOW_OFF,
	    FLOW_HARDWARE,
	    FLOW_XONXOFF
	} FlowType;

	typedef enum _ParityType {
	    PAR_NONE,
	    PAR_ODD,
	    PAR_EVEN
	} ParityType;

	typedef enum _DataBitsType {
	    DATA_5,
	    DATA_6,
	    DATA_7,
	    DATA_8
	} DataBitsType;

	typedef enum _StopBitsType {
	    STOP_1,
	    STOP_2
	} StopBitsType;

	typedef enum _BaudRateType {
	    BAUD50,                //POSIX ONLY
	    BAUD75,                //POSIX ONLY
	    BAUD110,
	    BAUD134,               //POSIX ONLY
	    BAUD150,               //POSIX ONLY
	    BAUD200,               //POSIX ONLY
	    BAUD300,
	    BAUD600,
	    BAUD1200,
	    BAUD1800,              //POSIX ONLY
	    BAUD2400,
	    BAUD4800,
	    BAUD9600,
	    //BAUD14400,             //WINDOWS ONLY
	    BAUD19200,
	    BAUD38400,
	    //BAUD56000,             //WINDOWS ONLY
	    BAUD57600,
	    BAUD76800,             //POSIX ONLY
	    BAUD115200,
		BAUD230400
	    //BAUD128000,            //WINDOWS ONLY
	    //BAUD256000             //WINDOWS ONLY
	} BaudRateType;

	/*structure to contain port settings*/
	typedef struct _PortSettings
	{
		FlowType flowControl;
		ParityType parity;
		DataBitsType dataBits;
		StopBitsType stopBits;
		BaudRateType baudRate;

		_PortSettings()
		{
			flowControl=FLOW_OFF;
			parity=PAR_NONE;
			dataBits=DATA_8;
			stopBits=STOP_1;
			baudRate=BAUD9600;
		}
	}
	PortSettings;

protected:
	fd_set rfds;
	struct timeval tv;

	bool portOpen;
	QString portName;
	PortSettings settings;
public:
	QFile portFile;
	int portDesc;
	Lector *lector;
	
protected slots:
	void slotNotifierActivated();

public:
	struct termios portConfig;

	QSerialPort();
	virtual ~QSerialPort();

	bool open(const QString& name);
	inline bool isOpen() const {return portOpen;}

	// QIODevice
	virtual bool open( int block = 0); //0 no block
	virtual void close();
	virtual void flush();
	virtual qint64 size() const;
        // Read/Write at most maxlen bytes ( see QIODevice )
	virtual qint64 read( char* data, qint64 maxlen );
	virtual qint64 readData( char* data, qint64 maxlen );
	qint64 readLine(char* data, qint64 maxlen);
	virtual qint64 write(  char* data, qint64 maxlen );
	virtual qint64 writeData( const char *data, qint64 maxlen);
	virtual int getch();
	virtual int putch(QChar ch);
	void setName(const QString& name);

	const QString& name() const;

	// Serial port.
	virtual void setFlowControl(FlowType);
	inline virtual FlowType flowControl() const {return settings.flowControl;}
	virtual void setParity(ParityType);
	inline virtual ParityType parity() const {return settings.parity;}
	virtual void setDataBits(DataBitsType);
	inline virtual DataBitsType dataBits() const {return settings.dataBits;}
	virtual void setStopBits(StopBitsType);
	inline virtual StopBitsType stopBits() const {return settings.stopBits;}
	virtual void setBaudRate(BaudRateType);
	inline virtual BaudRateType baudRate() const {return settings.baudRate;}
	virtual void setDtr(bool set=true);
	virtual void setRts(bool set=true);
	int getStatusBits() const;

private:
	QMutex mutex;

signals:
	void readyRead(int);
};



#endif
