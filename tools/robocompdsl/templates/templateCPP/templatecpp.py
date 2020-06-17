import datetime

from ..common.abstracttemplatesmanager import ComponentTemplatesManager
from ..templateCPP.functions import servant

DSR_UI_STR = """\
  <widget class="QWidget" name="centralwidget">
  <layout class="QVBoxLayout" name="verticalLayout">
  <item>
   <widget class="QSplitter" name="splitter_1">
    <property name="orientation">
     <enum>Qt::Horizontal</enum>
    </property>
    <widget class="QSplitter" name="splitter_2">
     <property name="sizePolicy">
      <sizepolicy hsizetype="Minimum" vsizetype="Expanding">
       <horstretch>0</horstretch>
       <verstretch>0</verstretch>
      </sizepolicy>
     </property>
     <property name="orientation">
      <enum>Qt::Vertical</enum>
     </property>
     <widget class="QTableWidget" name="tableWidgetNodes">
      <property name="sizePolicy">
       <sizepolicy hsizetype="Minimum" vsizetype="Expanding">
        <horstretch>0</horstretch>
        <verstretch>0</verstretch>
       </sizepolicy>
      </property>
     </widget>
     <widget class="QTableWidget" name="tableWidgetEdges">
      <property name="sizePolicy">
       <sizepolicy hsizetype="Minimum" vsizetype="Expanding">
        <horstretch>0</horstretch>
        <verstretch>0</verstretch>
       </sizepolicy>
      </property>
     </widget>
    </widget>
    <widget class="QScrollArea" name="scrollArea">
     <property name="sizePolicy">
      <sizepolicy hsizetype="Maximum" vsizetype="Expanding">
       <horstretch>0</horstretch>
       <verstretch>0</verstretch>
      </sizepolicy>
     </property>
     <property name="minimumSize">
      <size>
       <width>500</width>
       <height>0</height>
      </size>
     </property>
     <property name="widgetResizable">
      <bool>true</bool>
     </property>
     <widget class="QWidget" name="scrollAreaWidgetContents">
      <property name="geometry">
       <rect>
        <x>0</x>
        <y>0</y>
        <width>498</width>
        <height>443</height>
       </rect>
      </property>
     </widget>
    </widget>
   </widget>
  </item>
  </layout>
  </widget>
  <widget class="QMenuBar" name="menubar">
  <property name="geometry">
  <rect>
   <x>0</x>
   <y>0</y>
   <width>780</width>
   <height>25</height>
  </rect>
  </property>
  <widget class="QMenu" name="menuFile">
  <property name="title">
   <string>File</string>
  </property>
  <addaction name="actionSave"/>
  </widget>
  <widget class="QMenu" name="menuSimulation">
  <property name="title">
   <string>Simulation</string>
  </property>
  <addaction name="actionStart_Stop"/>
  </widget>
  <addaction name="menuFile"/>
  <addaction name="menuSimulation"/>
  </widget>
  <widget class="QStatusBar" name="statusbar"/>
  <action name="actionSave">
  <property name="text">
  <string>Save</string>
  </property>
  </action>
  <action name="actionStart_Stop">
  <property name="text">
  <string>Start/Stop</string>
  </property>
  </action>
"""

DSR_READ_CONFIG = """\
RoboCompCommonBehavior::Parameter aux;
configGetString( "","agent_name", aux.value,"");
params["agent_name"] = aux;
configGetString( "","agent_id", aux.value,"false");
params["agent_id"] = aux;
configGetString( "","read_dsr", aux.value,"true");
params["read_dsr"] = aux;
configGetString( "","dsr_input_file", aux.value, "");
params["dsr_input_file"] = aux;
"""

DSR_SOURCES_STR = """\
$ENV{ROBOCOMP}/classes/graph-related-classes/CRDT.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/CRDT_graphviewer.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/CRDT_graphnode.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/CRDT_graphedge.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/fast_rtps/dsrpublisher.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/fast_rtps/dsrsubscriber.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/fast_rtps/dsrparticipant.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/topics/DSRGraphPubSubTypes.cxx
$ENV{ROBOCOMP}/classes/graph-related-classes/topics/DSRGraph.cxx
$ENV{ROBOCOMP}/classes/graph-related-classes/inner_api.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/dsr_utils.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/dsr_to_osg_viewer.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/dsr_to_graphicscene_viewer.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/dsr_to_graph_viewer.cpp
$ENV{ROBOCOMP}/classes/graph-related-classes/dsr_to_tree_viewer.cpp
"""

DSR_HEADERS_STR = """\
$ENV{ROBOCOMP}/classes/graph-related-classes/CRDT.h
$ENV{ROBOCOMP}/classes/graph-related-classes/CRDT_graphviewer.h
$ENV{ROBOCOMP}/classes/graph-related-classes/CRDT_graph.h
$ENV{ROBOCOMP}/classes/graph-related-classes/CRDT_graphnode.h
$ENV{ROBOCOMP}/classes/graph-related-classes/CRDT_graphedge.h
"""

DSR_DEDINITIONS = """\
add_definitions(-O3  -fmax-errors=1 -std=c++2a )
SET(SPECIFIC_LIBS OpenThreads fastcdr fastrtps osgDB)
"""

class TemplatesManagerCpp(ComponentTemplatesManager):
    def __init__(self, component):
        self.files = {
            'regular': [
                'CMakeLists.txt', 'DoxyFile', 'README-RCNODE.txt', 'README.md', 'etc/config', 'src/main.cpp',
                'src/CMakeLists.txt', 'src/CMakeListsSpecific.txt', 'src/commonbehaviorI.h', 'src/commonbehaviorI.cpp',
                'src/genericmonitor.h', 'src/genericmonitor.cpp', 'src/config.h', 'src/specificmonitor.h',
                'src/specificmonitor.cpp', 'src/genericworker.h', 'src/genericworker.cpp', 'src/specificworker.h',
                'src/specificworker.cpp', 'src/mainUI.ui'
            ],
            'avoid_overwrite': [
                'src/specificworker.h', 'src/specificworker.cpp', 'src/CMakeListsSpecific.txt',
                'src/mainUI.ui', 'src/specificmonitor.h', 'src/specificmonitor.cpp', 'README.md',
                'etc/config'
            ],
            'servant_files': ["SERVANT.H", "SERVANT.CPP"],
            'template_path': "templateCPP/files/"
        }
        super(TemplatesManagerCpp, self).__init__(component)

    def SERVANT_H(self, interface_name):
        module = self.component.idsl_pool.module_providing_interface(interface_name)
        return {
            'year': str(datetime.date.today().year),
            'interface_name': interface_name,
            'interface_name_upper': interface_name.upper(),
            'filename_without_extension': module['filename'].split('/')[-1].split('.')[0],
            'module_name': module['name'],
            'interface_methods_definition': servant.interface_methods_definition(self.component,
                                                                                 module,
                                                                                 interface_name)
        }

    def SERVANT_CPP(self, interface_name):
        return {
            'year': str(datetime.date.today().year),
            'interface_name': interface_name,
            'interface_name_lower': interface_name.lower(),
            'interface_methods_creation': servant.interface_methods_creation(self.component, interface_name)
        }

    def README_md(self):
        return {
            'component_name': self.component.name
        }

    def DoxyFile(self):
        return {
            'component_name': self.component.name
        }

    def CMakeLists_txt(self):
        return {
            'component_name': self.component.name
        }

    def src_mainUI_ui(self):
        if self.component.dsr:
            dsr_widget=DSR_UI_STR
        else:
            dsr_widget = ""

        return {'gui_type': self.component.gui.widget,
                'component_name': self.component.name,
                'dsr_widget': dsr_widget}

    def src_config_h(self):
        need_gui = ""
        if self.component.gui is not None:
            need_gui = "#define USE_QTGUI\n\n"
        return {
            'component_name': self.component.name,
            'need_gui': need_gui
        }

    def src_commonbehaviorI_h(self):
        if self.component.language.lower() == 'cpp':
            const = "const"
            ampersand = "&"
        else:
            const = ""
            ampersand = ""
        return {
            'year': str(datetime.date.today().year),
            'const': const,
            'ampersand': ampersand
        }

    def src_commonbehaviorI_cpp(self):
        if self.component.language.lower() == 'cpp':
            const = "const"
            ampersand = "&"
        else:
            const = ""
            ampersand = ""
        return {
            'year': str(datetime.date.today().year),
            'const': const,
            'ampersand': ampersand
        }

    def src_specificmonitor_cpp(self):
        dsr_read_config = ""
        if self.component.dsr:
            dsr_read_config = DSR_READ_CONFIG
        return {
            'dsr_read_config': dsr_read_config,
        }

    def src_CMakeListsSpecific_txt(self):
        dsr_sources = ""
        dsr_headers = ""
        dsr_definitions = ""
        if self.component.dsr:
            dsr_sources = DSR_SOURCES_STR
            dsr_headers = DSR_HEADERS_STR
            dsr_definitions = DSR_DEDINITIONS
        return {'dsr_sources': dsr_sources,
                'dsr_headers': dsr_headers,
                'dsr_definitions': dsr_definitions}

