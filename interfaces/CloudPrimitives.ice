/** \mainpage RoboComp Interfaces: CloudPrimitives.ice
 *
 * \section intro_sec Introduction
* Interface for cloudprimitivesComp.
*
*
*
*    PORT  <br>
*/
#ifndef CLOUDPRIMITIVES_ICE
#define CLOUDPRIMITIVES_ICE

#include <IMU.ice>
#include <DifferentialRobot.ice>
#include <JointMotor.ice>
#include <RGBD.ice>

/** \namespace RoboCompCloudPrimitives
  *@brief Name space CloudPrimitives
  */
module RoboCompCloudPrimitives
{
  struct Point {
    float x;
    float y;
    float z;
  };

  sequence<Point> PointCloud;

  struct OrientedPatch {
    float a;
    float b;
    float c;
    float d;
    float eigenvalue;
    int points;
    int x;
    int y;
  };
  sequence<OrientedPatch> OrientedPatchList;

  sequence< float > Histogram;


  /** \interface CloudPrimitives
  *@brief interface CloudPrimitives
  */
  interface CloudPrimitives
  {
    void getHistogram(out Histogram hist, out RoboCompIMU::DataImu imuData, out RoboCompJointMotor::MotorStateMap hState, out RoboCompDifferentialRobot::TBaseState bState);
    void getData(out OrientedPatchList ptchs, out Histogram hist, out RoboCompRGBD::PointSeq points, out RoboCompIMU::DataImu imuData, out RoboCompJointMotor::MotorStateMap hState, out RoboCompDifferentialRobot::TBaseState bState);
    void getClusteredData(out OrientedPatchList ptchs, out Histogram hist, out RoboCompRGBD::PointSeq points, out RoboCompIMU::DataImu imuData, out RoboCompJointMotor::MotorStateMap hState, out RoboCompDifferentialRobot::TBaseState bState);
    bool setWindowRadius(int radius);
    int getWindowRadius();
    void setActive(bool active);
  };


};

#endif
