set(SUBSYS_NAME components)
set(SUBSYS_DESC "RoboComp InnerModel library")

INCLUDE( laserComp/component_config.cmake )
RoboComp_ADD_COMPONENT( laserComp "${laser_interfaces}" "${laser_headers}" ${laser_src} )
