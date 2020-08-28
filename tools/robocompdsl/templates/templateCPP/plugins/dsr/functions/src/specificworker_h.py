import datetime

import dsl_parsers.parsing_utils as p_utils
from templates.templateCPP.plugins.base.functions import function_utils as utils
from templates.common.templatedict import TemplateDict

DSR_INCLUDES_STR = """\
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
"""

DSR_ATTRIBUTES = """\
// DSR graph
std::shared_ptr<DSR::DSRGraph> G;

//DSR params
std::string agent_name;
int agent_id;

bool tree_view;
bool graph_view;
bool qscene_2d_view;
bool osg_3d_view;

// DSR graph viewer
std::unique_ptr<DSR::DSRViewer> graph_viewer;
QHBoxLayout mainLayout;
"""

class specificworker_h(TemplateDict):
    def __init__(self, component):
        super(specificworker_h, self).__init__()
        self.component = component
        self['dsr_includes'] = self.dsr_includes()
        self['dsr_attributes'] = self.dsr_attributes()



    def dsr_includes(self):
        result = ""
        if self.component.dsr:
            result = DSR_INCLUDES_STR
        return result

    def dsr_attributes(self):
        result=""
        if self.component.dsr:
            result = DSR_ATTRIBUTES
        return result


