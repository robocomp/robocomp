import datetime
from string import Template

import dsl_parsers.parsing_utils as p_utils
from robocompdsl.templates.templateCPP.plugins.base.functions import function_utils as utils
from robocompdsl.templates.common.templatedict import TemplateDict

DSR_READ_CONFIG = """\
RoboCompCommonBehavior::Parameter aux;
configGetString( "","agent_name", aux.value,"");
params["agent_name"] = aux;
configGetString( "","agent_id", aux.value,"false");
params["agent_id"] = aux;

configGetString( "","tree_view", aux.value, "none");
params["tree_view"] = aux;
configGetString( "","graph_view", aux.value, "none");
params["graph_view"] = aux;
configGetString( "","2d_view", aux.value, "none");
params["2d_view"] = aux;
configGetString( "","3d_view", aux.value, "none");
params["3d_view"] = aux;
"""

class src_specificmonitor_cpp(TemplateDict):
    def __init__(self, component):
        super(src_specificmonitor_cpp, self).__init__()
        self.component = component
        dsr_read_config = ""
        if self.component.dsr:
            dsr_read_config = DSR_READ_CONFIG
        self['dsr_read_config'] = dsr_read_config
