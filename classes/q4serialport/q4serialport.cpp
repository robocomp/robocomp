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
#include "q4serialport.h"

QSerialPort::QSerialPort() : portOpen(false)
{
}

QSerialPort::~QSerialPort()
{
}

bool QSerialPort::open(const QString& name)
{
	if(portOpen)
		return true;

	setName(name);

	if ( open( QIODevice::ReadWrite ) )
	{
	  tv.tv_sec = 1;
	  tv.tv_usec = 0;
	  FD_ZERO(&rfds);
	  FD_SET(portDesc, &rfds);
	  return true;
	}
	else
	  return false;
}

bool QSerialPort::open(int block)
{
	if( block== 1) portDesc = ::open(portName.toLatin1(),O_RDWR);
	else portDesc = ::open(portName.toLatin1(),O_RDWR | O_NONBLOCK );
	if(portDesc == -1)
	{
		return false;
	}
	portOpen = true;

	tcgetattr(portDesc, &portConfig);
	
	portConfig.c_cflag|=(CLOCAL | CREAD);
	portConfig.c_lflag&=(~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|ISIG));
	portConfig.c_iflag&=(~(IGNBRK|BRKINT|PARMRK|ISTRIP |INLCR| IGNCR|ICRNL|IXON)); //  (~(INPCK|IGNPAR|PARMRK|ISTRIP|IXANY));
	portConfig.c_iflag&=~(ICRNL);

	portConfig.c_oflag&=(~OPOST);
	
	tcsetattr(portDesc, TCSANOW, &portConfig);

	setBaudRate(settings.baudRate);
	setDataBits(settings.dataBits);
	setFlowControl(settings.flowControl);
	setParity(settings.parity);
	setStopBits(settings.stopBits);
	setDtr();
	setRts();

	connect(&portFile, SIGNAL(readyRead()), this, SLOT(slotNotifierActivated()));

	return true;
}

void QSerialPort::close()
{
	if(!portOpen)
		return;
	::close(portDesc);
	portOpen=false;
	
 	disconnect(&portFile, SIGNAL(readyRead()), this, SLOT(slotNotifierActivated()));
}

void QSerialPort::slotNotifierActivated()
{
	int nbytes;
	nbytes=size();
	printf( "caca\n");
	if(nbytes>0) // Esto siempre debe ocurrir...
		emit readyRead(nbytes);
}

void QSerialPort::flush()
{
	char data[100];

	if(!portOpen)
		return;
	while(size()>0)
	{
		if(size() < 100) read(data,size());
		else read(data, 100);
	}
}

qint64 QSerialPort::size() const
{
	int nbytes;
	if(!portOpen)
		return 0;

	if(ioctl(portDesc, FIONREAD, &nbytes)<0)
		return 0;
	return nbytes;
}

//Lectura No Bloqueante
qint64 QSerialPort::read(char* data, qint64 maxlen)
{
    int retval=1;
	int nBytes=0, leidos=0;

	if(!portOpen)
        return 0;

	QMutexLocker locker(&mutex);
	QTime time;
	time.start();

	tv.tv_sec = 1;
	tv.tv_usec = 0;


	while (nBytes<maxlen && leidos != -1 && time.elapsed() < 1000)
	{
		FD_ZERO(&rfds);
		FD_SET(portDesc, &rfds);

		retval = select(portDesc+1, &rfds, NULL, NULL, &tv);
		if (retval==0)
		{
			continue;
		}
		else if (retval==-1)
		{
			switch(errno)
			{
			case EBADF: printf("An invalid file descriptor was given in one of the sets.\n"); break;
			case EINTR: printf("A signal was caught.\n"); break;
			case EINVAL: printf("nfds is negative or the value contained within timeout is invalid.\n"); break;
			case ENOMEM: printf("unable to allocate memory for internal tables.\n"); break;
			default: printf("??\n");
			}
			qDebug()<<"Read SerialPort: value=-1";	
			return(0);
		}
		else
		{
			if (retval > maxlen-nBytes)
			{
				
				leidos = ::read(portDesc, data+nBytes, maxlen-nBytes);
				if (leidos != maxlen-nBytes)
				{
// 					printf("leidos != maxlen-nBytes     (%d, %lld)!!\n", leidos, maxlen-nBytes);
				}
			}
			else
			{
				
				leidos = ::read(portDesc, data+nBytes, retval);
				if (leidos != retval)
				{
// 					printf("leidos != retval     (%d, %d)!!\n", leidos, retval);
				}
			}


			if (leidos == -1)
			{
				switch(errno)
				{
					case EINTR: printf("The call has been interrupted by a signal before any data has been read.\n"); break;
					case EAGAIN: printf("Non-blocking I/O has been selected using O_NONBLOCK and there was no data\n"); break;
					case EIO: printf("I/O error. This can happen for example when the process is in a group of pr\n"); break;
					case EISDIR: printf("fd refers to a directory.\n"); break;
					case EBADF: printf("fd is not a valid file descriptor or is not open for reading.\n"); break;
					case EINVAL: printf("fd is associated with an object that is not appropriate for reading.\n"); break;
					case EFAULT: printf("buff is outside the address space accessible to the user.\n"); break;
					default: printf("??\n");
				}
			}
			else
				nBytes += leidos;
		}
	}
	return nBytes;
}

qint64 QSerialPort::readData(char * data, qint64 maxlen)
{
	return read(data, maxlen);
}

qint64 QSerialPort::readLine(char * data, qint64 maxlen)
{
	QMutexLocker locker(&mutex);
    int nBytes=0,leidos=0;

	if(!portOpen)
        return 0;
	do
		{
			leidos = ::read(portDesc, data+nBytes, 1);
			if(leidos > -1) nBytes += leidos;
		} while( ( data[nBytes-1] != 10 ) && ( leidos != -1 ) && ( nBytes < maxlen ) );

	data[nBytes] = '\0';
	return nBytes;
}

qint64 QSerialPort::write(  char* data, qint64 maxlen)
{
	QMutexLocker locker(&mutex);

	int nBytes=0;

	if(!portOpen)
	    return 0;
	while(nBytes<maxlen && nBytes != -1)
		nBytes += ::write(portDesc, data+nBytes, maxlen-nBytes);

  	return nBytes;
}

qint64 QSerialPort::writeData(const char * data, qint64 maxlen)
{
	return write(const_cast<char*> (data), maxlen);
}

int QSerialPort::getch()
{
    char c;
	if(!portOpen)
		return 0;

	if(read(&c, 1)==-1) return -1;
	else
	{
	  return (int)c;
	}
}

int QSerialPort::putch(QChar ch)
{
	if(!portOpen)
		return 0;

	if(write((char *)&ch,static_cast<qint64>(sizeof(QChar)))==-1) return -1;
	else return 0;
}


void QSerialPort::setName( const QString& name )
{
	if( portOpen )
		return;

	portName=name;
}

const QString& QSerialPort::name() const
{
    return portName;
}

void QSerialPort::setBaudRate(QSerialPort::BaudRateType baud)
{
	struct termios portConfig;
	BaudRateType res=baud;

	if(!portOpen)
	{
		settings.baudRate=baud;
		return;
	}
	tcgetattr(portDesc, &portConfig);

	switch(baud)
	{
	case BAUD50:
		cfsetispeed(&portConfig, B50);
		cfsetospeed(&portConfig, B50);
		break;
	case BAUD75:
		cfsetispeed(&portConfig, B75);
		cfsetospeed(&portConfig, B75);
		break;
	case BAUD110:
		cfsetispeed(&portConfig, B110);
		cfsetospeed(&portConfig, B110);
		break;
	case BAUD134:
		cfsetispeed(&portConfig, B134);
		cfsetospeed(&portConfig, B134);
		break;
	case BAUD150:
		cfsetispeed(&portConfig, B150);
		cfsetospeed(&portConfig, B150);
		break;
	case BAUD200:
		cfsetispeed(&portConfig, B200);
		cfsetospeed(&portConfig, B200);
		break;
	case BAUD300:
		cfsetispeed(&portConfig, B300);
		cfsetospeed(&portConfig, B300);
		break;
	case BAUD600:
		cfsetispeed(&portConfig, B600);
		cfsetospeed(&portConfig, B600);
		break;
	case BAUD1200:
		cfsetispeed(&portConfig, B1200);
		cfsetospeed(&portConfig, B1200);
		break;
	case BAUD1800:
		cfsetispeed(&portConfig, B1800);
		cfsetospeed(&portConfig, B1800);
		break;
	case BAUD2400:
		cfsetispeed(&portConfig, B2400);
		cfsetospeed(&portConfig, B2400);
		break;
	case BAUD4800:
		cfsetispeed(&portConfig, B4800);
		cfsetospeed(&portConfig, B4800);
		break;
	case BAUD9600:
		cfsetispeed(&portConfig, B9600);
		cfsetospeed(&portConfig, B9600);
		break;
	case BAUD19200:
		cfsetispeed(&portConfig, B19200);
		cfsetospeed(&portConfig, B19200);
		break;
	case BAUD38400:
		cfsetispeed(&portConfig, B38400);
		cfsetospeed(&portConfig, B38400);
		break;
	case BAUD57600:
		cfsetispeed(&portConfig, B57600);
		cfsetospeed(&portConfig, B57600);
		break;
	case BAUD76800:
#ifndef B76800
		std::cerr<< "Error: Speed 76800 not supported. Changing to 57600.";
		cfsetispeed(&portConfig, B57600);
		cfsetospeed(&portConfig, B57600);
		res=BAUD57600;
#else
		cfsetispeed(&portConfig, B76800);
		cfsetospeed(&portConfig, B76800);
#endif
		break;
	case BAUD115200:
		cfsetispeed(&portConfig, B115200);
		cfsetospeed(&portConfig, B115200);
		break;
	case BAUD230400:
		cfsetispeed(&portConfig, B230400);
		cfsetospeed(&portConfig, B230400);
		break;
	default:
		cfsetispeed(&portConfig, B9600);
		cfsetospeed(&portConfig, B9600);
		break;
	}
	if(tcsetattr(portDesc, TCSAFLUSH, &portConfig)==0) settings.baudRate=res;
}

void QSerialPort::setDataBits(QSerialPort::DataBitsType bits)
{
	struct termios portConfig;
	DataBitsType res=bits;

	if(!portOpen)
	{
		settings.dataBits=bits;
		return;
	}

	tcgetattr(portDesc, &portConfig);

	portConfig.c_cflag&=~CSIZE;			// Se restablecen a 0 los bits que indican el tama� del car�er.
	switch(bits)
	{
	case DATA_5:
		portConfig.c_cflag|=CS5;
		break;
	case DATA_6:
		portConfig.c_cflag|=CS6;
		break;
	case DATA_7:
		portConfig.c_cflag|=CS7;
		break;
	case DATA_8:
		portConfig.c_cflag|=CS8;
		break;
	}

 	if(tcsetattr(portDesc, TCSAFLUSH, &portConfig)==0) settings.dataBits=res;
}

void QSerialPort::setFlowControl(QSerialPort::FlowType flow)
{
	struct termios portConfig;
	FlowType res=flow;

	if(!portOpen)
	{
		settings.flowControl=flow;
		return;
	}

	tcgetattr(portDesc, &portConfig);

	switch(flow)
	{
	case FLOW_OFF:
		portConfig.c_cflag&=(~CRTSCTS);
		portConfig.c_iflag&=(~(IXON|IXOFF|IXANY));
		break;

	case FLOW_XONXOFF:
		portConfig.c_cflag&=(~CRTSCTS);
		portConfig.c_iflag|=(IXON|IXOFF|IXANY);
		break;

	case FLOW_HARDWARE:
		portConfig.c_cflag |= CRTSCTS;
		portConfig.c_iflag &= (~(IXON|IXOFF|IXANY));
		break;
	}

	if(tcsetattr(portDesc, TCSAFLUSH, &portConfig)==0) settings.flowControl=res;
}

void QSerialPort::setParity(QSerialPort::ParityType parity)
{
	struct termios portConfig;
	ParityType res=parity;

	if(!portOpen)
	{
		settings.parity=parity;
		return;
	}

	tcgetattr(portDesc, &portConfig);

	if(parity==PAR_NONE)
		portConfig.c_cflag&=~PARENB;
	else
	{
		portConfig.c_cflag|=PARENB;
		if(parity==PAR_ODD)
			portConfig.c_cflag|=PARODD;
		else
			portConfig.c_cflag&=~PARODD;
	}

	if(tcsetattr(portDesc, TCSAFLUSH, &portConfig)==0)
		settings.parity=res;
}

void QSerialPort::setStopBits(QSerialPort::StopBitsType stop)
{
	struct termios portConfig;
	StopBitsType res=stop;

	if(!portOpen)
	{
		settings.stopBits=stop;
		return;
	}

	tcgetattr(portDesc, &portConfig);
		
	if(stop==STOP_1)
		portConfig.c_cflag&=~CSTOPB;
	else
		portConfig.c_cflag|=CSTOPB;
	
	if(tcsetattr(portDesc, TCSAFLUSH, &portConfig)==0)
		settings.stopBits=res;	
}

void QSerialPort::setDtr(bool set)
{
	int statusBits;
	if(!portOpen)
		return;
		
	ioctl(portDesc, TIOCMGET, &statusBits);
		
	if(set)
		statusBits|=TIOCM_DTR;
	else
		statusBits&=~TIOCM_DTR;
	
	ioctl(portDesc, TIOCMSET, &statusBits);
}

void QSerialPort::setRts(bool set)
{
	int statusBits;
	if(!portOpen)
		return;
		
	ioctl(portDesc, TIOCMGET, &statusBits);

	if(set)
		statusBits|=TIOCM_RTS;
	else
		statusBits&=~TIOCM_RTS;
	
	ioctl(portDesc, TIOCMSET, &statusBits);

}

int QSerialPort::getStatusBits() const
{
	int statusBits;
	ioctl(portDesc, TIOCMGET, &statusBits);
	return statusBits;
}

