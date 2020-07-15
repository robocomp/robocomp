import datetime
from string import Template

import dsl_parsers.parsing_utils as p_utils
from .. import function_utils as utils

AGM_INNERMODEL_ASSOCIATION_STR = """\
innerModel = std::make_shared<InnerModel>(new InnerModel());
try
{
	RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
	AGMExecutiveTopic_structuralChange(w);
}
catch(...)
{
	printf("The executive is probably not running, waiting for first AGM model publication...");
}
"""

COMPUTE_METHOD_STR = """\
void SpecificWorker::compute()
{
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	${compute_innermodelviewer}
	
}
"""


INNERMODEL_COMPUTE_STR = """\
#ifdef USE_QTGUI
    if (innerModelViewer) innerModelViewer->update();
    osgView->frame();
#endif
"""


STATEMACHINE_WITH_COMPUTE_METHOD = """
void SpecificWorker::sm_${state}()
{
	std::cout<<\"Entered state ${state}\"<<std::endl;
	compute();
}

"""

STATEMACHINE_METHOD = """
void SpecificWorker::sm_${state}()
{
	std::cout<<\"Entered ${type}state ${state}\"<<std::endl;
}

"""

REGENERATE_INNERMODEL = """
void SpecificWorker::regenerateInnerModelViewer()
{
    if (innerModelViewer)
    {
        osgView->getRootGroup()->removeChild(innerModelViewer);
    }

    innerModelViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
}   
    
"""

SET_PARAMETERS_AND_POSSIBLE_ACTIVATION = """
bool SpecificWorker::setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (RoboCompAGMCommonBehavior::ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\\n");

	return true;
}
"""

SEND_MODIFICATION_PROPOSAL = """
void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMMisc::publishModification(newModel, ${proxy}, \"${agent_name}Agent\");
	}
/*	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
	}
*/
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}
"""

INTERFACE_TYPES_COMMENT_STR = """\
/**************************************/
// From the ${module_name} you can use this types:
${types}
"""

PROXY_METHODS_COMMENT_STR = """\
/**************************************/
// From the ${module_name} you can ${action} this methods:
${methods}
"""

DSR_SET_PARAMS = """\
agent_name = params["agent_name"].value;
agent_id = stoi(params["agent_id"].value);
read_dsr = params["read_dsr"].value == "true";
dsr_input_file = params["dsr_input_file"].value;

tree_view = params["tree_view"].value == "true";
graph_view = params["graph_view"].value == "true";
qscene_2d_view = params["2d_view"].value == "true";
osg_3d_view = params["3d_view"].value == "true";
"""

DSR_INITIALIZE = """\
// create graph
G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

// Graph viewer
using opts = DSR::GraphViewer::view;
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
graph_viewer = std::make_unique<DSR::GraphViewer>(this, G, current_opts, main);
setWindowTitle(QString::fromStdString(agent_name + "-" + dsr_input_file));

this->Period = period;
timer.start(Period);
"""

class TemplateDict(dict):
    def __init__(self, component):
        super(TemplateDict, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['proxy_map_type'] = self.proxy_map_type()
        self['proxy_map_name'] = self.proxy_map_name()
        self['innermodelviewer_code'] = self.innermodelviewer_code()
        self['agmagent_attributes'] = self.agmagent_attributes()
        self['statemachine_finalize_emit'] = self.statemachine_finalize_emit()
        self['innermodel_and_viewer_attribute_init'] = self.innermodel_and_viewer_attribute_init()
        self['agm_innermodel_association'] = self.agm_innermodel_association()
        self['state_machine_start'] = self.state_machine_start()
        self['statemachine_initialize_to_compute'] = self.statemachine_initialize_to_compute()
        self['compute_method'] = self.compute_method()
        self['statemachine_methods_creation'] = self.statemachine_methods_creation()
        self['implements'] = self.implements()
        self['subscribes'] = self.subscribes()
        self['agm_specific_code'] = self.agm_specific_code()
        self['interface_specific_comment'] = self.interface_specific_comment()
        self['dsr_destructor'] = self.dsr_destructor()
        self['dsr_set_params'] = self.dsr_set_params()
        self['dsr_initialize'] = self.dsr_initialize()

    # TODO: Extract pieces of code to strings and refactor
    def body_code_from_name(self, name):
        body_code = ""
        if self.component.is_agm1_agent():
            #######################################################
            # code to implement subscription to AGMExecutiveTopic #
            #######################################################
            if name == 'symbolUpdated':
                body_code = "\tQMutexLocker locker(mutex);\n\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n"
            elif name == 'symbolsUpdated':
                body_code = "\tQMutexLocker l(mutex);\n\tfor (auto modification : modifications)\n\t\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n"
            elif name == 'edgeUpdated':
                body_code = "\tQMutexLocker locker(mutex);\n\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n\tAGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());\n"
            elif name == 'edgesUpdated':
                body_code = "\tQMutexLocker lockIM(mutex);\n\tfor (auto modification : modifications)\n\t{\n\t\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n\t\tAGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());\n\t}\n"
            elif name == 'structuralChange':
                body_code = "\tQMutexLocker lockIM(mutex);\n \tAGMModelConverter::fromIceToInternal(w, worldModel);\n \n\tinnerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel));"
                if 'innermodelviewer' in [x.lower() for x in self.component.options]:
                    body_code += "\n\tregenerateInnerModelViewer();"
            elif name == 'selfEdgeAdded':
                body_code = "\tQMutexLocker lockIM(mutex);\n \ttry { worldModel->addEdgeByIdentifiers(nodeid, nodeid, edgeType, attributes); } catch(...){ printf(\"Couldn't add an edge. Duplicate?\\n\"); }\n \n\ttry { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf(\"Can't extract an InnerModel from the current model.\\n\"); }"
                if 'innermodelviewer' in [x.lower() for x in self.component.options]:
                    body_code += "\n\tregenerateInnerModelViewer();"
            elif name == 'selfEdgeDeleted':
                body_code = "\tQMutexLocker lockIM(mutex);\n \ttry { worldModel->removeEdgeByIdentifiers(nodeid, nodeid, edgeType); } catch(...) { printf(\"Couldn't remove an edge\\n\"); }\n \n\ttry { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf(\"Can't extract an InnerModel from the current model.\\n\"); }"
                if 'innermodelviewer' in [x.lower() for x in self.component.options]:
                    body_code += "\n\tregenerateInnerModelViewer();"
            #######################################
            # code to implement AGMCommonBehavior #
            #######################################
            elif name == 'activateAgent':
                body_code = "\tbool activated = false;\n\tif (setParametersAndPossibleActivation(prs, activated))\n\t{\n\t\tif (not activated)\n\t\t{\n\t\t\treturn activate(p);\n\t\t}\n\t}\n\telse\n\t{\n\t\treturn false;\n\t}\n\treturn true;"
            elif name == 'deactivateAgent':
                body_code = "\treturn deactivate();"
            elif name == 'getAgentState':
                body_code = "\tRoboCompAGMCommonBehavior::StateStruct s;\n\tif (isActive())\n\t{\n\t\ts.state = RoboCompAGMCommonBehavior::StateEnum::Running;\n\t}\n\telse\n\t{\n\t\ts.state = RoboCompAGMCommonBehavior::StateEnum::Stopped;\n\t}\n\ts.info = p.action.name;\n\treturn s;"
            elif name == 'getAgentParameters':
                body_code = "\treturn params;"
            elif name == 'setAgentParameters':
                body_code = "\tbool activated = false;\n\treturn setParametersAndPossibleActivation(prs, activated);"
            elif name == 'uptimeAgent':
                body_code = "\treturn 0;"
            elif name == 'reloadConfigAgent':
                body_code = "\treturn true;"

        elif self.component.is_agm2_agent():
            mdlw = 'Ice'
            if name == 'symbolsUpdated':
                body_code = "\tQMutexLocker locker(mutex);\n\tfor (auto n : modification"
                body_code += ")\n\t\tAGMModelConverter::include" + mdlw + "ModificationInInternalModel(n, worldModel);\n"
            elif name == 'edgesUpdated':
                body_code = "\tQMutexLocker locker(mutex);\n\tfor (auto e : modification"
                body_code += ")\n\t{\n\t\tAGMModelConverter::include" + mdlw + "ModificationInInternalModel(e, worldModel);\n\t\tAGMInner::updateImNodeFromEdge(worldModel, e, innerModelViewer->innerModel.get());\n\t}\n"
            elif name == 'structuralChange':
                body_code = "\tQMutexLocker locker(mutex);\n\tAGMModelConverter::from" + mdlw + "ToInternal(w, worldModel);\n\t\n\tInnerModel *newIM = AGMInner::extractInnerModel(worldModel);\n"
                if 'innermodelviewer' in self.component.options:
                    body_code += "\tif (innerModelViewer)\n\t{\n\t\tosgView->getRootGroup()->removeChild(innerModelViewer);\n\t\tdelete innerModel;\n\t}\n"
                body_code += "\tinnerModel = newIM;\n"
                if 'innermodelviewer' in self.component.options:
                    body_code += "\tinnerModelViewer = new InnerModelViewer(innerModel, \"root\", osgView->getRootGroup(), true);\n"

        return body_code

    def innermodelviewer_code(self):
        result = ""
        if self.component.innermodelviewer:
            result += "#ifdef USE_QTGUI\n"
            result += "<TABHERE>innerModelViewer = NULL;\n"
            result += "<TABHERE>osgView = new OsgView(this);\n"
            result += "<TABHERE>osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;\n"
            result += "<TABHERE>osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));\n"
            result += "<TABHERE>osg::Vec3d center(osg::Vec3(0.,0.,-0.));\n"
            result += "<TABHERE>osg::Vec3d up(osg::Vec3(0.,1.,0.));\n"
            result += "<TABHERE>tb->setHomePosition(eye, center, up, true);\n"
            result += "<TABHERE>tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));\n"
            result += "<TABHERE>osgView->setCameraManipulator(tb);\n"
            result += "#endif\n"
        return result

    def agmagent_attributes(self):
        result = ""
        if self.component.is_agm1_agent() or self.component.is_agm2_agent():
            result += "active = false;\n"
            result += "worldModel = AGMModel::SPtr(new AGMModel());\n"
            result += "worldModel->name = \"worldModel\";\n"
        return result

    def innermodel_and_viewer_attribute_init(self):
        result = ""
        if self.component.innermodelviewer:
            result += "#ifdef USE_QTGUI\n"
            result += "<TABHERE>innerModel = std::make_shared<InnerModel>(); //InnerModel creation example\n"
            result += "<TABHERE>innerModelViewer = new InnerModelViewer (innerModel, \"root\", osgView->getRootGroup(), true);\n"
            result += "#endif\n"
        return result

    def agm_innermodel_association(self):
        result = ""

        if self.component.is_agm1_agent():
            result += AGM_INNERMODEL_ASSOCIATION_STR
        elif self.component.is_agm2_agent():
            result += "// TODO: Here we should ask the DSR for the current model for initialization purposes.\n"

        return result

    def compute_method(self):
        result = ""
        statemachine = self.component.statemachine
        if (statemachine is not None and statemachine['machine']['default'] is True) or self.component.statemachine_path is None:
            result += Template(COMPUTE_METHOD_STR).substitute(compute_innermodelviewer=self.compute_innermodelviewer())
        return result


    def compute_innermodelviewer(self):
        result = ""
        if self.component.innermodelviewer:
            result += INNERMODEL_COMPUTE_STR
        return result

    def statemachine_methods_creation(self):
        sm_implementation = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            sm_implementation = "\n"
            state_type = ""
            if statemachine['machine']['contents']['states'] is not None:
                for state in statemachine['machine']['contents']['states']:
                    if statemachine['machine']['default'] and state == 'compute':
                        sm_implementation += Template(STATEMACHINE_WITH_COMPUTE_METHOD).substitute(state=state, type=state_type)
                    else:
                        sm_implementation += Template(STATEMACHINE_METHOD).substitute(state=state, type=state_type)
            if statemachine['machine']['contents']['initialstate'] is not None:
                state_type = "initial "
                state = statemachine['machine']['contents']['initialstate']
                sm_implementation += Template(STATEMACHINE_METHOD).substitute(state=state, type=state_type)
            if statemachine['machine']['contents']['finalstate'] is not None:
                state_type = "final "
                state = statemachine['machine']['contents'][
                    'finalstate']
                sm_implementation += Template(STATEMACHINE_METHOD).substitute(state=state, type=state_type)
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['states'] is not None:
                        state_type = "sub"
                        for state in substates['contents']['states']:
                            sm_implementation += Template(STATEMACHINE_METHOD).substitute(state=state, type=state_type)
                    if substates['contents']['initialstate'] is not None:
                        state_type = "initial sub"
                        state = substates['contents']['initialstate']
                        sm_implementation += Template(STATEMACHINE_METHOD).substitute(state=state, type=state_type)
                    if substates['contents']['finalstate'] is not None:
                        state_type = "final sub"
                        state = substates['contents']['finalstate']
                        sm_implementation += Template(STATEMACHINE_METHOD).substitute(state=state, type=state_type)
            sm_implementation += '\n'
        return sm_implementation

    def implements(self):
        result = ""
        pool = self.component.idsl_pool
        for impa in self.component.implements:
            if type(impa) == str:
                imp = impa
            else:
                imp = impa[0]
            module = pool.module_providing_interface(imp)
            for interface in module['interfaces']:
                if interface['name'] == imp:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        param_str_a = ''
                        body_code = self.body_code_from_name(method['name'])
                        if p_utils.communication_is_ice(impa):
                            param_str_a = utils.get_parameters_string(method, module['name'], self.component.language)
                            return_type = utils.get_type_string(method['return'], module['name'])
                            result += return_type + ' SpecificWorker::' + interface['name'] + "_" + method[
                                'name'] + '(' + param_str_a + ")\n{\n//implementCODE\n" + body_code + "\n}\n\n"
                        else:
                            pass
        return result

    def subscribes(self):
        result = ""
        pool = self.component.idsl_pool
        for subscribes in self.component.subscribesTo:
            module = pool.module_providing_interface(subscribes.name)
            if module is None:
                raise ValueError('\nCan\'t find module providing %s\n' % subscribes.name)
            for interface in module['interfaces']:
                if interface['name'] == subscribes.name:
                    for mname in interface['methods']:
                        method = interface['methods'][mname]
                        param_str_a = ''
                        body_code = self.body_code_from_name(method['name'])
                        if p_utils.communication_is_ice(subscribes):
                            param_str_a = utils.get_parameters_string(method, module['name'], self.component.language)
                            result += "//SUBSCRIPTION to " + method['name'] + " method from " + interface[
                                'name'] + " interface\n"
                            result += method['return'] + ' SpecificWorker::' + interface['name'] + "_" + method[
                                'name'] + '(' + param_str_a + ")\n{\n//subscribesToCODE\n" + body_code + "\n}\n\n"
                        else:
                            pass
        return result

    def agm_specific_code(self):
        result = ""
        if ('agmagent' in [x.lower() for x in self.component.options]) and (
                'innermodelviewer' in [x.lower() for x in self.component.options]):
            result += REGENERATE_INNERMODEL

        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += SET_PARAMETERS_AND_POSSIBLE_ACTIVATION
            agent_name = self.component.name
            if self.component.language.lower() == "cpp":
                proxy = "agmexecutive_proxy"
            else:
                proxy = "*agmexecutive_proxy.get()"
            result += Template(SEND_MODIFICATION_PROPOSAL).substitute(proxy=proxy, agent_name=agent_name)
        return result

    def proxy_map_type(self):
        if self.component.language.lower() == 'cpp':
            return "MapPrx&"
        else:
            return "TuplePrx"

    def proxy_map_name(self):
        if self.component.language.lower() == 'cpp':
            return "mprx"
        else:
            return "tprx"

    def statemachine_finalize_emit(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None and statemachine['machine']['default']:
            result += "emit t_compute_to_finalize();\n"
        return result

    def state_machine_start(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            result += statemachine['machine']['name'] + ".start();\n"
        return result

    def statemachine_initialize_to_compute(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None and statemachine['machine']['default']:
            result += "emit this->t_initialize_to_compute();\n"
        return result

    def interface_specific_comment(self):
        result = ""
        interfaces_by_type = {
            "requires": self.component.requires,
            "publishes":  self.component.publishes,
            "implements": self.component.implements,
            "subscribesTo": self.component.subscribesTo
        }
        for interface_type, interfaces in interfaces_by_type.items():
            for interface, num in p_utils.get_name_number(interfaces):
                if p_utils.communication_is_ice(interface):
                    proxy_methods_calls = ""
                    module = self.component.idsl_pool.module_providing_interface(interface.name)
                    if interface_type in ["publishes", "requires"]:
                        if interface_type == 'publishes':
                            action = "publish calling"
                            pub = "pub"
                        else:
                            action = "call"
                            pub = ""
                        proxy_reference = "this->" + interface.name.lower() + num + f"_{pub}proxy->"
                        for method in module['interfaces'][0]['methods']:
                            proxy_methods_calls += f"// {proxy_reference}{method}(...)\n"
                        if proxy_methods_calls:
                            result += Template(PROXY_METHODS_COMMENT_STR).substitute(module_name=module['name'],
                                                                                     methods=proxy_methods_calls,
                                                                                     action=action)
                    structs_str = ""
                    for struct in module['structs']:
                        structs_str += f"// {struct['name'].replace('/', '::')}\n"
                    if structs_str:
                        result += Template(INTERFACE_TYPES_COMMENT_STR).substitute(module_name=module['name'],
                                                                                   types=structs_str)
        return result

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