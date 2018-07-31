INCLUDE_DIRECTORIES( $ENV{ASTRA_SDK_INCLUDE} )
SET( LIBS ${LIBS} -L$ENV{ASTRA_SDK_LIB} -lastra -lastra_core_api -lastra_core)

