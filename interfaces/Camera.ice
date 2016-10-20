#ifndef ROBOCOMPCAMERA_ICE
#define ROBOCOMPCAMERA_ICE

#include <CommonHead.ice>

#include <GenericBase.ice>

module RoboCompCamera{
	exception HardwareFailedException{string what;};
	exception MovingImageException{string what;};
	sequence <byte> imgType;
	sequence <int> intVector;
	["cpp:comparable"]
	struct TCamParams{
		int focal;
		int width;
		int height;
		int size;
		int numCams;
		int FPS;
		int timerPeriod;
		int leftCamera;
		int rightCamera;
		int bothCameras;
		int inverted;
		int rotated;
		int leftInverted;
		int rightInverted;
		int saturation;
		int lineFreq;
		bool talkToBase;
		bool talkToJointMotor;
		string name;
		string driver;
		string device;
		string mode;
	};

	interface Camera{
		idempotent 
		void  getYUVImage(int cam, out imgType roi, out RoboCompCommonHead::THeadState hState, out RoboCompGenericBase::TBaseState bState)throws HardwareFailedException;
		idempotent 
		void  getYImage(int cam, out imgType roi, out RoboCompCommonHead::THeadState hState, out RoboCompGenericBase::TBaseState bState)throws MovingImageException;
		idempotent 
		void  getYLogPolarImage(int cam, out imgType roi, out RoboCompCommonHead::THeadState hState, out RoboCompGenericBase::TBaseState bState)throws MovingImageException;
		idempotent 
		void  getYImageCR(int cam, int div, out imgType roi, out RoboCompCommonHead::THeadState hState, out RoboCompGenericBase::TBaseState bState)throws MovingImageException;
		idempotent 
		void  getRGBPackedImage(int cam, out imgType roi, out RoboCompCommonHead::THeadState hState, out RoboCompGenericBase::TBaseState bState)throws MovingImageException;
		idempotent 
		void  getYRGBImage(int cam, out imgType roi, out RoboCompCommonHead::THeadState hState, out RoboCompGenericBase::TBaseState bState)throws MovingImageException;
		TCamParams getCamParams();
		idempotent 
		void  setInnerImage(imgType roi);
	};
};
  
#endif