import datetime
from string import Template

import robocompdsl.dsl_parsers.parsing_utils as p_utils
from robocompdsl.templates.templateCPP.plugins.base.functions import function_utils as utils
from robocompdsl.templates.common.templatedict import TemplateDict

DSR_SET_PARAMS = """\
try
{
	agent_name = params.at("agent_name").value;
	agent_id = stoi(params.at("agent_id").value);
	tree_view = params.at("tree_view").value == "true";
	graph_view = params.at("graph_view").value == "true";
	qscene_2d_view = params.at("2d_view").value == "true";
	osg_3d_view = params.at("3d_view").value == "true";
}
catch(const std::exception &e){ std::cout << e.what() << " Error reading params from config file" << std::endl;};
"""

DSR_INITIALIZE = """\
// create graph
G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

//dsr update signals
connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_node_attrs_slot);
connect(G.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &SpecificWorker::modify_edge_attrs_slot);
connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

// Graph viewer
using opts = DSR::DSRViewer::view;
int current_opts = 0;
opts main = opts::none;
if(tree_view)
{
    current_opts = current_opts | opts::tree;
}
if(graph_view)
{
    current_opts = current_opts | opts::graph;
    main = opts::graph;
}
if(qscene_2d_view)
{
    current_opts = current_opts | opts::scene;
}
if(osg_3d_view)
{
    current_opts = current_opts | opts::osg;
}
graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

/***
Custom Widget
In addition to the predefined viewers, Graph Viewer allows you to add various widgets designed by the developer.
The add_custom_widget_to_dock method is used. This widget can be defined like any other Qt widget,
either with a QtDesigner or directly from scratch in a class of its own.
The add_custom_widget_to_dock method receives a name for the widget and a reference to the class instance.
***/
//graph_viewer->add_custom_widget_to_dock("CustomWidget", &custom_widget);

this->Period = period;
timer.start(Period);
"""

class specificworker_cpp(TemplateDict):
    def __init__(self, component):
        super(specificworker_cpp, self).__init__()
        self.component = component
        self['dsr_destructor'] = self.dsr_destructor()
        self['dsr_set_params'] = self.dsr_set_params()
        self['dsr_initialize'] = self.dsr_initialize()

    def dsr_destructor(self):
        result = ""
        if self.component.dsr:
            result = "G->write_to_json_file(\"./\"+agent_name+\".json\");\nG.reset();\n"
        return result

    def dsr_set_params(self):
        result = ""
        if self.component.dsr:
            result += DSR_SET_PARAMS
        return result

    def dsr_initialize(self):
        result = ""
        if self.component.dsr:
            result += DSR_INITIALIZE
        return result
