IF(NOT EXISTS $ENV{OPENNI2})
MESSAGE(FATAL_ERROR "
 OPENNI2 variable not set, check your bashrc profile!
 example, XXX means your architecture.
 OPENNI2=/home/user/openni_dir/OpenNI-Linux-XXX-2.2
")
ENDIF(NOT EXISTS $ENV{OPENNI2})

INCLUDE_DIRECTORIES( $ENV{OPENNI2}/Include )
LINK_DIRECTORIES($ENV{OPENNI2}/Redist)
LINK_DIRECTORIES($ENV{OPENNI2}/Bin/x64-Release)
LIST ( APPEND LIBS OpenNI2 )


