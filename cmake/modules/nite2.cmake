IF(NOT EXISTS $ENV{NITE2})
MESSAGE(FATAL_ERROR "
 NITE2 variable not set, check your bashrc profile!
 example, XXX means your architecture.
 NITE2=/home/user/nite_dir/NiTe-Linux-XXX-2.2
")
ENDIF(NOT EXISTS $ENV{NITE2})

INCLUDE_DIRECTORIES( $ENV{NITE2}/Include )
LINK_DIRECTORIES($ENV{NITE2}/Redist)
LIST ( APPEND LIBS NiTE2 )


