import datetime

import dsl_parsers.parsing_utils as p_utils
from templates.templateCPP.plugins.base.functions import function_utils as utils
from templates.common.templatedict import TemplateDict

DSR_INCLUDES_STR = """\
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
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

DSR_SLOTS = """\
void add_or_assign_node_slot(std::uint64_t, const std::string &type){};
void add_or_assign_attrs_slot(std::uint64_t id, const std::map<std::string, DSR::Attribute> &attribs){};
void add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};

void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
void del_node_slot(std::uint64_t from){};     
"""

class specificworker_h(TemplateDict):
    def __init__(self, component):
        super(specificworker_h, self).__init__()
        self.component = component
        self['dsr_includes'] = self.dsr_includes()
        self['dsr_attributes'] = self.dsr_attributes()
        self['dsr_slots'] = self.dsr_slots()


    def dsr_includes(self):
        result = ""
        if self.component.dsr:
            result = DSR_INCLUDES_STR
        return result

    def dsr_attributes(self):
        result = ""
        if self.component.dsr:
            result = DSR_ATTRIBUTES
        return result

    def dsr_slots(self):
        result = ""
        if self.component.dsr:
            result = DSR_SLOTS
        return result

