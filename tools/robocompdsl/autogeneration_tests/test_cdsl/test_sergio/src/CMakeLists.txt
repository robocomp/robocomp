cmake_minimum_required(VERSION 2.6)
PROJECT(  real_sense_pose_estimation )

INCLUDE( /opt/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior FullPoseEstimation FullPoseEstimationPub )
