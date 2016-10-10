#ifndef QJOYSTICK_H
#define QJOYSTICK_H

#include <sys/ioctl.h>

#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <linux/joystick.h>
#include <linux/input.h>
#include <QIODevice>
#include <QFile>
#include <QThread>
#include <QtCore>

/**
  * @author Ricardo Royo Anton
  */

// Joystick definitions
#define DEFAULT_DEVICE "/dev/input/js0"

#define JOYSTICK_VERSION_NAME_LENGTH 128


class QJoyStick : public QThread
{
Q_OBJECT
public:
	// Joystick data structure

	// Joystick info structure
	struct qjs_info_t {
	uint8_t axes;
	uint8_t buttons;
	int32_t version;
	char name[128];
	};

	QJoyStick(QString Device=DEFAULT_DEVICE, int32_t axes=2, QObject *parent = 0);
	~QJoyStick();

	bool openQJoy();
	static bool cmpJoyEv( js_event src, js_event dst );

	std::string getDeviceName() { return deviceName.toStdString(); }
private:
	QString deviceName;
	int fd;
	qjs_info_t info;
	js_event data;
	int data_sz;

	int srate;

public slots:
	void run();
	void stop();

signals:
	void inputEvent( int value, int type, int number );
};

#endif
