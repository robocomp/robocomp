from dsl_parsers.parsing_utils import communication_is_ice, is_agm1_agent, is_agm2_agent


def interface_sources(component):
    result = ""
    #TODO: refactor in one loop
    for ima in component.implements:
        if type(ima) == str:
            im = ima
        else:
            im = ima[0]
        if communication_is_ice(ima):
            result += im.lower() + 'I.cpp\n'

    for subscribe in component.subscribesTo:
        subs = subscribe
        while type(subs) != type(''):
            subs = subs[0]
        if communication_is_ice(subscribe):
            result += subs.lower() + 'I.cpp\n'
    return result

STATEMACHINE_VISUAL_SOURCES_STR = """
$ENV{ROBOCOMP}/classes/statemachinewidget/edge.cpp
$ENV{ROBOCOMP}/classes/statemachinewidget/node.cpp
$ENV{ROBOCOMP}/classes/statemachinewidget/graphwidget.cpp
$ENV{ROBOCOMP}/classes/statemachinewidget/qstateMachineWrapper.cpp
"""

def statemachine_visual_sources(statemachine_visual):
    result = ""
    if statemachine_visual:
        result += STATEMACHINE_VISUAL_SOURCES_STR
    return result

ROS_INCLUDES_STR = """
#ROS
find_package(catkin REQUIRED)
include_directories(  ${catkin_INCLUDE_DIRS} )
include_directories(  "/opt/ros/melodic/include" )
SET ( LIBS ${LIBS} ${catkin_LIBRARIES} -L/opt/ros/melodic/lib -lroscpp -lrosconsole -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lrosconsole_backend_interface)
"""

def ros_includes(using_ros):
    result = ""
    if using_ros is True:
        result += ROS_INCLUDES_STR
    return result

CPP11_ICE_STR = """
ADD_DEFINITIONS ("-DICE_CPP11_MAPPING")
FIND_PACKAGE( Ice REQUIRED COMPONENTS Ice++11 IceStorm++11)
"""

def cpp11_ice_packages(language):
    result = ""
    if language.lower() == "cpp11":
        result+=CPP11_ICE_STR
    return result


AGM_INCLUDES_STR = """
# AGM Agent\'s requirements
find_package(LibXml2 REQUIRED)
include_directories(LIBXML2_INCLUDE_DIRS)
include_directories(/usr/include/libxml2/)
"""

def agm_includes(component):
    result = ""
    if 'agmagent' in [x.lower() for x in component.options]:
        result += AGM_INCLUDES_STR
    try:
        if is_agm1_agent(component):
            result += 'SET(LIBS ${LIBS} -lagm)\n'
            result += 'ADD_DEFINITIONS( -I/usr/include/libxml2/)\n'
        if is_agm2_agent(component):
            result += 'SET(LIBS ${LIBS} -lagm2)\n'
            result += 'ADD_DEFINITIONS( -I/usr/include/libxml2/)\n'
            result += 'include(/usr/local/share/cmake/FindAGM2.cmake)\n'
    except:
        print("Can't check if the component is an agent")
        pass
    return result

def wrap_ice(component):
    result = """ROBOCOMP_WRAP_ICE( CommonBehavior """

    iface_names = []
    for im in component.recursiveImports + component.ice_interfaces_names:
        name = im.split('/')[-1].split('.')[0]
        iface_names.append(name)
    result += ' '.join(iface_names)
    try:
        options = [x.lower() for x in component.options]
        if 'agmagent' in options:
            result +=  " AGMExecutive AGMExecutiveTopic "
        elif 'agm2agent' in options or 'agm2agentICE' in options or 'agm2agentROS' in options:
            result +=  " AGM2 "
    except:
        pass
    result += ")"
    return result
