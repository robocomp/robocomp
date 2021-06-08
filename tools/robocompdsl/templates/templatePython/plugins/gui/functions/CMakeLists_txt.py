from templates.common.templatedict import TemplateDict

WRAP_PYTHON_UI_STR="""
MACRO( WRAP_PYTHON_UI )
  FOREACH( input_file ${ARGN} )
    ADD_CUSTOM_COMMAND (
      OUTPUT src/ui_${input_file}.py
      COMMAND pyside2-uic ${CMAKE_CURRENT_SOURCE_DIR}/src/${input_file}.ui -o ${CMAKE_CURRENT_SOURCE_DIR}/src/ui_${input_file}.py
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${input_file}.ui
      COMMENT "Generating src/ui_${input_file}.py from src/${input_file}.ui"
    )
    ADD_CUSTOM_TARGET(ui_${input_file} ALL DEPENDS src/ui_${input_file}.py )
  ENDFOREACH( input_file )
ENDMACRO( WRAP_PYTHON_UI )
WRAP_PYTHON_UI( mainUI )
"""

class CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super(CMakeLists_txt, self).__init__()
        self.component = component
        if self.component.gui is not None:
            wrap_python_ui = WRAP_PYTHON_UI_STR
        else:
            wrap_python_ui = ""
        self['wrap_python_ui'] = wrap_python_ui

