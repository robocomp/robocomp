cmake_minimum_required( VERSION 3.10 )
project(rcnode
        VERSION 2021.2.1
        DESCRIPTION "RoboComp rcnode tool"
        LANGUAGES CXX )


# Program destination directory
SET( DEST_DIR "/opt/robocomp/")

INSTALL(FILES rcnode.sh   DESTINATION "${DEST_DIR}/bin/" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE RENAME rcnode)
INSTALL(FILES rcnode.conf DESTINATION "${DEST_DIR}/etc/" PERMISSIONS OWNER_READ OWNER_WRITE               GROUP_READ               WORLD_READ               )
