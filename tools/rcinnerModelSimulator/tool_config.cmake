QT4_WRAP_UI( RCIS_UI_HEADERS RCInnerModelSimulator/src/guiDlg.ui )

set( rcis_interfaces
	Camera
	CommonBehavior
	CommonHead
	DifferentialRobot
	IMU
	InnerModelManager
	JointMotor
	Laser
	RGBD
	Display
  )

set ( rcis_src
	RCInnerModelSimulator/src/robotsimulatorcomp.cpp
	RCInnerModelSimulator/src/genericworker.cpp
	RCInnerModelSimulator/src/genericmonitor.cpp
	RCInnerModelSimulator/src/cameraI.cpp
	RCInnerModelSimulator/src/differentialrobotI.cpp
	RCInnerModelSimulator/src/imuI.cpp
	RCInnerModelSimulator/src/innermodelmanagerI.cpp
	RCInnerModelSimulator/src/jointmotorI.cpp
	RCInnerModelSimulator/src/laserI.cpp
	RCInnerModelSimulator/src/rgbdI.cpp
	RCInnerModelSimulator/src/displayI.cpp
	$ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
	$ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Header files
set ( rcis_headers
	RCInnerModelSimulator/src/genericworker.h
	RCInnerModelSimulator/src/genericmonitor.h
	RCInnerModelSimulator/src/cameraI.h
	RCInnerModelSimulator/src/differentialrobotI.h
	RCInnerModelSimulator/src/imuI.h
	RCInnerModelSimulator/src/innermodelmanagerI.h
	RCInnerModelSimulator/src/jointmotorI.h
	RCInnerModelSimulator/src/laserI.h
	RCInnerModelSimulator/src/rgbdI.h
	RCInnerModelSimulator/src/displayI.h
	${RCIS_UI_HEADERS}
)
