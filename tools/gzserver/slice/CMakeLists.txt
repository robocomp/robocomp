execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp GenericBase.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp Laser.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp Motors.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp containers.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/IMU.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/DifferentialRobot.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/

)

execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/Camera.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/RGBD.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/bumper.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/JointMotor.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/CommonHead.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp/
)

execute_process(
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} 
    COMMAND slice2cpp ${CMAKE_CURRENT_LIST_DIR}/Exceptions.ice --output-dir ${CMAKE_CURRENT_LIST_DIR}/../slice_cpp
)
