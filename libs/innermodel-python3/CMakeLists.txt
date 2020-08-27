set(LIB_NAME innermodel-python3)
set(LIB_DESC "RoboComp innermodel-python3 library")
set(OUT_FILE_NAME ${PROJECT_NAME_LOWER}_${LIB_NAME})

execute_process (
    COMMAND bash -c "cd ${RoboComp_LIBS_DIR}/innermodel-python3 && python3 ${RoboComp_LIBS_DIR}/innermodel-python3/setup.py install --user"
)
