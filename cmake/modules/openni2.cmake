IF(NOT EXISTS $ENV{OPENNI2_INCLUDE})
MESSAGE(FATAL_ERROR "
 OPENNI2_INCLUDE variables not set, check your bashrc profile!
 example, XXX means your architecture.
 OPENNI2_INCLUDE=/home/user/openni_dir/OpenNI-Linux-XXX-2.2/lib
")
ENDIF(NOT EXISTS $ENV{OPENNI2_INCLUDE})

IF(NOT EXISTS $ENV{OPENNI2_LINK})
MESSAGE(FATAL_ERROR "
 OPENNI2_LINK variables not set, check your bashrc profile!
 example, XXX means your architecture.
 OPENNI2_LINK=/home/user/openni_dir/OpenNI-Linux-XXX-2.2/lib
")
ENDIF(NOT EXISTS $ENV{OPENNI2_LINK})


INCLUDE_DIRECTORIES( $ENV{OPENNI2_INCLUDE}/Include )
INCLUDE_DIRECTORIES( $ENV{OPENNI2_INCLUDE} )
LINK_DIRECTORIES($ENV{OPENNI2_LINK}/OpenNI2/Drivers)
LINK_DIRECTORIES($ENV{OPENNI2_LINK})
LIST ( APPEND LIBS OpenNI2 )


