from dsl_parsers.parsing_utils import communication_is_ice
from templates.common.templatedict import TemplateDict

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
class src_CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        if self.component.dsr:
            dsr_widget = DSR_UI_STR
        else:
            dsr_widget = ""
        self['dsr_widget'] = dsr_widget

