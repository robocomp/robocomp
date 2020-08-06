from dsl_parsers.parsing_utils import communication_is_ice
from templates.common.templatedict import TemplateDict

DSR_SOURCES_STR = """\
$ENV{ROBOCOMP}/classes/dsr/core/rtps/dsrpublisher.cpp
$ENV{ROBOCOMP}/classes/dsr/core/rtps/dsrsubscriber.cpp
$ENV{ROBOCOMP}/classes/dsr/core/rtps/dsrparticipant.cpp
$ENV{ROBOCOMP}/classes/dsr/core/topics/IDLGraphPubSubTypes.cxx
$ENV{ROBOCOMP}/classes/dsr/core/topics/IDLGraph.cxx
$ENV{ROBOCOMP}/classes/dsr/api/dsr_api.cpp
$ENV{ROBOCOMP}/classes/dsr/api/dsr_inner_api.cpp
$ENV{ROBOCOMP}/classes/dsr/api/dsr_utils.cpp
$ENV{ROBOCOMP}/classes/dsr/gui/dsr_gui.cpp
$ENV{ROBOCOMP}/classes/dsr/gui/viewers/osg_3d_viewer/osg_3d_viewer.cpp
$ENV{ROBOCOMP}/classes/dsr/gui/viewers/qscene_2d_viewer/qscene_2d_viewer.cpp
$ENV{ROBOCOMP}/classes/dsr/gui/viewers/graph_viewer/graph_viewer.cpp
$ENV{ROBOCOMP}/classes/dsr/gui/viewers/graph_viewer/graph_node.cpp
$ENV{ROBOCOMP}/classes/dsr/gui/viewers/graph_viewer/graph_edge.cpp
$ENV{ROBOCOMP}/classes/dsr/gui/viewers/tree_viewer/tree_viewer.cpp
$ENV{ROBOCOMP}/classes/dsr/gui/viewers/_abstract_graphic_view.cpp
"""

DSR_HEADERS_STR = """\
$ENV{ROBOCOMP}/classes/dsr/api/dsr_api.h
$ENV{ROBOCOMP}/classes/dsr/gui/dsr_gui.h
"""

DSR_DEDINITIONS = """\
add_definitions(-g  -fmax-errors=1 -std=c++2a )
"""

DSR_LIBS = " fastcdr fastrtps osgDB"


class src_CMakeListsSpecific_txt(TemplateDict):
    def __init__(self, component):
        super().__init__()
        dsr_sources = ""
        dsr_headers = ""
        dsr_definitions = ""
        dsr_libs = ""
        if component.dsr:
            dsr_sources = DSR_SOURCES_STR
            dsr_headers = DSR_HEADERS_STR
            dsr_definitions = DSR_DEDINITIONS
            dsr_libs = DSR_LIBS
        self['dsr_sources'] = dsr_sources
        self['dsr_headers'] = dsr_headers
        self['dsr_definitions'] = dsr_definitions
        self['dsr_libs'] = dsr_libs