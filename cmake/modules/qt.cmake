# QT

#option (USE_QT5 "Build with QT5" ON)
#option (USE_QT4 "Build with QT4" OFF)

SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTSQL TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
SET( CMAKE_AUTOMOC ON )
SET( CMAKE_AUTOUIC TRUE )
ADD_DEFINITIONS( "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB" )




#find_package(Qt5)
#if(QT5_FOUND)#
#	MESSAGE( STATUS "COMPILING WITH QT5 =  1" )
#else (QT5_SUPPORT)
#	find_package(Qt4)
#	if ()
#	MESSAGE( STATUS "COMPILING WITHOUT QT5 = 0" )
#endif(QT5_FOUND)


find_package(Qt4)
if (QT4_FOUND)
INCLUDE(${QT_USE_FILE})
  MACRO(QT_WRAP_CPP output )
	FOREACH( input_file ${ARGN} )
		QT4_WRAP_CPP( ${output} ${input_file} )
	ENDFOREACH( input_file)
	SET_PROPERTY(SOURCE ${${output}} PROPERTY SKIP_AUTOGEN ON)
  ENDMACRO(QT_WRAP_CPP)
  macro (QT_WRAP_UI outfile inputfile )
	QT4_WRAP_UI( outfile inputfile )
  endmacro(QT_WRAP_UI)
endif (QT4_FOUND)

#find_package(Qt5 REQUIRED COMPONENTS Gui )

#if (QT5_FOUND)
#  message(status "********* Qt5 FOUND" )
#  MACRO(QT_WRAP_CPP output )
#	FOREACH( input_file ${ARGN} )
#		QT5_WRAP_CPP( ${output} ${input_file} )
#	ENDFOREACH( input_file)
#	SET_PROPERTY(SOURCE ${${output}} PROPERTY SKIP_AUTOGEN ON)
#  ENDMACRO(QT_WRAP_CPP)
#  macro (QT_WRAP_UI outfile inputfile )
#	QT4_WRAP_UI( outfile inputfile )
#  endmacro(QT_WRAP_UI)
#endif (QT5_FOUND)

#message(status "*********" ${Qt5Widgets_INCLUDES})


