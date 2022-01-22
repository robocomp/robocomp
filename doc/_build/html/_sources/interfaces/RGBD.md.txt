# RGBD.idsl

```
import "JointMotor.idsl";
import "DifferentialRobot.idsl";

module RoboCompRGBD
{
  exception HardwareFailedException { string what; };

  /** @brief RGB-Depth registration method */
  enum Registration { None, DepthInColor, ColorInDepth };

  sequence<byte> imgType;
  sequence<float> depthType;

  struct ColorRGB {
    byte red;
    byte green;
    byte blue;
  };

  struct PointXYZ {
    float x;
    float y;
    float z;
    float w;
  };

  sequence<float> DepthSeq;
  sequence<ColorRGB> ColorSeq;
  sequence<PointXYZ> PointSeq;

  struct CameraParameters
  {
    int focal;
    int width;
    int height;
    int size;
    int FPS;
  };

  /** \struct TRGBDParams
  *@brief struct RGBD parameters
  */
  struct TRGBDParams
  {
  	CameraParameters color;
    CameraParameters depth;
    int timerPeriod;
    bool talkToBase;
    bool talkToJointMotor;
    string driver;
    string device;
  };

  /** \interface RGBD
  *@brief interface RGBD
  */
  interface RGBD
  {
    /**@brief Return RGBD parameters
    * @return struct TRGBDParams
    */
    TRGBDParams getRGBDParams();

    /**@brief Return RGBD parameters
    * @return struct TRGBDParams
    */
    //idempotent TRGBDParams getParams() throws HardwareFailedException;

    /** @brief Set registration method */
    idempotent void setRegistration (Registration value) throws HardwareFailedException;

    /** @brief Get LED Color */
    idempotent Registration getRegistration () throws HardwareFailedException;

    /**@brief RGB Packed - Distance 1 plane */
    idempotent void getData(out imgType rgbMatrix, out depthType distanceMatrix, out RoboCompJointMotor::MotorStateMap hState, out RoboCompDifferentialRobot::TBaseState bState) throws HardwareFailedException;

    idempotent void getDepthInIR(out depthType distanceMatrix, out RoboCompJointMotor::MotorStateMap hState, out RoboCompDifferentialRobot::TBaseState bState) throws HardwareFailedException;

    /** @brief Return the RGB and the depth images */
    idempotent void getImage(
        out ColorSeq color,
        out DepthSeq depth,
        out PointSeq points,
        out RoboCompJointMotor::MotorStateMap hState,
        out RoboCompDifferentialRobot::TBaseState bState) throws HardwareFailedException;

    void getDepth(
    	out DepthSeq depth,
        out RoboCompJointMotor::MotorStateMap hState,
        out RoboCompDifferentialRobot::TBaseState bState) throws HardwareFailedException;

    void getRGB(out ColorSeq color,
        out RoboCompJointMotor::MotorStateMap hState,
        out RoboCompDifferentialRobot::TBaseState bState) throws HardwareFailedException;

    void getXYZ(out PointSeq points,
        out RoboCompJointMotor::MotorStateMap hState,
        out RoboCompDifferentialRobot::TBaseState bState) throws HardwareFailedException;
  };
};
```

Here in this interface one can calculate the depth, Get an image and display it and also can get various parameters pertaining to an image.

## getData, getDepthinIR

This is used to get the RGB image matrix and it's depth. In RGBD.idsl this can be done by the following code,

	static RoboCompDifferentialRobot::TBaseState bState;
	static RoboCompJointMotor::MotorStateMap hState;
	RoboCompRGBD::imgType rgbMatrix;
	RoboCompRGBD::depthType distanceMatrix;
	rgbd_proxy->getData(rgbMatrix,distanceMatrix, hState, bState)
	rgbd_proxy->getDepthInIR(distanceMatrix, hState, bState)

What the above code does the first two lines defines the bot and gets the state of the simulated bot. The next two lines defines a matrix, rgbMatrix and another matrix, distanceMatrix which would contain the depth. The final line calls the function getData which takes all the above as parameters. If you are interested in only getting the distance then getDepthinIR can be used.

## getImage, getDepth, getXYZ, getRGB

This is used to get the depth, color of each points(co-ordinates) of an image. This can be achieved with the following

	static RoboCompDifferentialRobot::TBaseState bState;
	static RoboCompJointMotor::MotorStateMap hState;
	RoboCompRGBD::ColorSeq color;
	RoboCompRGBD::DepthSeq depth;
	RoboCompRGBD::PointSeq points;
	rgbd_proxy->getImage(color, depth, points, hState, bState)
	rgbd_proxy->getRGB(color, hState, bState)
	rgbd_proxy->getXYZ(points, hState, bState)
	rgbd_proxy->getDepth(depth, hState, bState)

To get the color or the RGB values of each point/pixel in a image getRGB is used. To get all the points and it's co-ordinates getXYZ is used and to get the depth or distance of these points getDepth is used. To get the all the three parameters getImage is used.
