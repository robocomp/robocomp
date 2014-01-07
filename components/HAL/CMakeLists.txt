INCLUDE( laserComp/cmake.in )

RoboComp_ADD_COMPONENT( laserComp "${laser_interfaces}" "${laser_headers}" ${laser_src} )
