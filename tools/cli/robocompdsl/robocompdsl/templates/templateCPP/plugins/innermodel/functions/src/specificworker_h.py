import datetime

import robocompdsl.dsl_parsers.parsing_utils as p_utils
from robocompdsl.templates.templateCPP.plugins.base.functions import function_utils as utils
from robocompdsl.templates.common.templatedict import TemplateDict

INNERMODELVIEWER_ATTRIBUTES_STR = """\
#ifdef USE_QTGUI
	OsgView *osgView;
	InnerModelViewer *innerModelViewer;
#endif
"""

INNERMODEL_ATTRIBUTE_STR = """\
std::shared_ptr < InnerModel > innerModel;
"""


INNERMODEL_INCLUDE_STR = """\
#include <innermodel/innermodel.h>
"""

INNERMODELVIEWER_INCLUDES_STR = """\
#ifdef USE_QTGUI
	#include <osgviewer/osgview.h>
	#include <innermodel/innermodelviewer.h>
#endif
"""

class specificworker_h(TemplateDict):
    def __init__(self, component):
        super(specificworker_h, self).__init__()
        self.component = component
        self['innermodel_include'] = self.innermodel_include()
        self['innermodelviewer_includes'] = self.innermodelviewer_includes()
        self['innermodel_attribute'] = self.innermodel_attribute()
        self['innermodelviewer_attributes'] = self.innermodelviewer_attributes()

    def innermodel_attribute(self):
        result = ""
        if "dsr" not in self.component.options:
            result += INNERMODEL_ATTRIBUTE_STR
        return result

    def innermodelviewer_attributes(self):
        result = ""
        if self.component.innermodelviewer:
            result += INNERMODELVIEWER_ATTRIBUTES_STR
        return result

    def innermodelviewer_includes(self):
        result = ""
        if "dsr" not in self.component.options:
            result += INNERMODEL_INCLUDE_STR
        return result

    def innermodel_include(self):
        result = ""
        if self.component.innermodelviewer:
            result += INNERMODELVIEWER_INCLUDES_STR
        return result


