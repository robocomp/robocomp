import datetime
from string import Template

import dsl_parsers.parsing_utils as p_utils


# TODO: Extract pieces of code to strings and refactor
def body_code_from_name(name, component):
    bodyCode = ""
    if component.is_agm1_agent():
        #######################################################
        # code to implement subscription to AGMExecutiveTopic #
        #######################################################
        if name == 'symbolUpdated':
            bodyCode = "\tQMutexLocker locker(mutex);\n\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n"
        elif name == 'symbolsUpdated':
            bodyCode = "\tQMutexLocker l(mutex);\n\tfor (auto modification : modifications)\n\t\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n"
        elif name == 'edgeUpdated':
            bodyCode = "\tQMutexLocker locker(mutex);\n\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n\tAGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());\n"
        elif name == 'edgesUpdated':
            bodyCode = "\tQMutexLocker lockIM(mutex);\n\tfor (auto modification : modifications)\n\t{\n\t\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n\t\tAGMInner::updateImNodeFromEdge(worldModel, modification, innerModel.get());\n\t}\n"
        elif name == 'structuralChange':
            bodyCode = "\tQMutexLocker lockIM(mutex);\n \tAGMModelConverter::fromIceToInternal(w, worldModel);\n \n\tinnerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel));"
            if 'innermodelviewer' in [x.lower() for x in component.options]:
                bodyCode += "\n\tregenerateInnerModelViewer();"
        elif name == 'selfEdgeAdded':
            bodyCode = "\tQMutexLocker lockIM(mutex);\n \ttry { worldModel->addEdgeByIdentifiers(nodeid, nodeid, edgeType, attributes); } catch(...){ printf(\"Couldn't add an edge. Duplicate?\\n\"); }\n \n\ttry { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf(\"Can't extract an InnerModel from the current model.\\n\"); }"
            if 'innermodelviewer' in [x.lower() for x in component.options]:
                bodyCode += "\n\tregenerateInnerModelViewer();"
        elif name == 'selfEdgeDeleted':
            bodyCode = "\tQMutexLocker lockIM(mutex);\n \ttry { worldModel->removeEdgeByIdentifiers(nodeid, nodeid, edgeType); } catch(...) { printf(\"Couldn't remove an edge\\n\"); }\n \n\ttry { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf(\"Can't extract an InnerModel from the current model.\\n\"); }"
            if 'innermodelviewer' in [x.lower() for x in component.options]:
                bodyCode += "\n\tregenerateInnerModelViewer();"
        #######################################
        # code to implement AGMCommonBehavior #
        #######################################
        elif name == 'activateAgent':
            bodyCode = "\tbool activated = false;\n\tif (setParametersAndPossibleActivation(prs, activated))\n\t{\n\t\tif (not activated)\n\t\t{\n\t\t\treturn activate(p);\n\t\t}\n\t}\n\telse\n\t{\n\t\treturn false;\n\t}\n\treturn true;"
        elif name == 'deactivateAgent':
            bodyCode = "\treturn deactivate();"
        elif name == 'getAgentState':
            bodyCode = "\tStateStruct s;\n\tif (isActive())\n\t{\n\t\ts.state = RoboCompAGMCommonBehavior::StateEnum::Running;\n\t}\n\telse\n\t{\n\t\ts.state = RoboCompAGMCommonBehavior::StateEnum::Stopped;\n\t}\n\ts.info = p.action.name;\n\treturn s;"
        elif name == 'getAgentParameters':
            bodyCode = "\treturn params;"
        elif name == 'setAgentParameters':
            bodyCode = "\tbool activated = false;\n\treturn setParametersAndPossibleActivation(prs, activated);"
        elif name == 'uptimeAgent':
            bodyCode = "\treturn 0;"
        elif name == 'reloadConfigAgent':
            bodyCode = "\treturn true;"

    elif component.is_agm2_agent():
        mdlw = 'Ice'
        if p_utils.is_agm2_agent_ROS(component):
            mdlw = 'ROS'
        elif name == 'symbolsUpdated':
            bodyCode = "\tQMutexLocker locker(mutex);\n\tfor (auto n : modification"
            if mdlw == 'ROS':
                bodyCode += ".NodeSequence"
            bodyCode += ")\n\t\tAGMModelConverter::include" + mdlw + "ModificationInInternalModel(n, worldModel);\n"
        elif name == 'edgesUpdated':
            bodyCode = "\tQMutexLocker locker(mutex);\n\tfor (auto e : modification"
            if mdlw == 'ROS':
                bodyCode += ".EdgeSequence"
            bodyCode += ")\n\t{\n\t\tAGMModelConverter::include" + mdlw + "ModificationInInternalModel(e, worldModel);\n\t\tAGMInner::updateImNodeFromEdge(worldModel, e, innerModelViewer->innerModel.get());\n\t}\n"
        elif name == 'structuralChange':
            bodyCode = "\tQMutexLocker locker(mutex);\n\tAGMModelConverter::from" + mdlw + "ToInternal(w, worldModel);\n\t\n\tInnerModel *newIM = AGMInner::extractInnerModel(worldModel);\n"
            if 'innermodelviewer' in component.options:
                bodyCode += "\tif (innerModelViewer)\n\t{\n\t\tosgView->getRootGroup()->removeChild(innerModelViewer);\n\t\tdelete innerModel;\n\t}\n"
            bodyCode += "\tinnerModel = newIM;\n"
            if 'innermodelviewer' in component.options:
                bodyCode += "\tinnerModelViewer = new InnerModelViewer(innerModel, \"root\", osgView->getRootGroup(), true);\n"

    return bodyCode


def innermodelviewer_code(innermodelviewer):
    result = ""
    if innermodelviewer:
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

def agmagent_attributes(component):
    result = ""
    try:
        if component.is_agm1_agent() or component.is_agm2_agent():
            result += "active = false;\n"
            result += "worldModel = AGMModel::SPtr(new AGMModel());\n"
            result += "worldModel->name = \"worldModel\";\n"

    except:
        pass
    return result

def innermodel_and_viewer_attribute_init(innermodelviewer):
    result = ""
    if innermodelviewer:
        result += "#ifdef USE_QTGUI\n"
        result += "<TABHERE>innerModel = std::make_shared<InnerModel>(); //InnerModel creation example\n"
        result += "<TABHERE>innerModelViewer = new InnerModelViewer (innerModel, \"root\", osgView->getRootGroup(), true);\n"
        result += "#endif\n"
    return result

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

def agm_innermodel_association(component):
    result = ""
    try:
        if component.is_agm1_agent():
            result += AGM_INNERMODEL_ASSOCIATION_STR
        elif component.is_agm2_agent():
            result += "// TODO: Here we should ask the DSR for the current model for initialization purposes.\n"
    except:
        pass
    return result

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
	${compute_ros}
	${compute_innermodelviewer}
}
"""

def compute_method(component, statemachine):
    result = ""
    if (statemachine is not None and statemachine['machine']['default'] is True) or component.statemachine_path is None:
        result += Template(COMPUTE_METHOD_STR).substitute(compute_ros=compute_ros(component),
                                                          compute_innermodelviewer=compute_innermodelviewer(component))
    return result

def compute_ros(component):
    if component.usingROS == True:
        return "ros::spinOnce();"
    else:
        return ""

INNERMODEL_COMPUTE_STR= """\
#ifdef USE_QTGUI
    if (innerModelViewer) innerModelViewer->update();
    osgView->frame();
#endif
"""

def compute_innermodelviewer(component):
    result = ""
    if component.innermodelviewer:
        result += INNERMODEL_COMPUTE_STR
    return result


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

def statemachine_methods_creation(statemachine):
    sm_implementation = ""
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

def implements(component):
    result = ""
    pool = component.idsl_pool
    for impa in component.implements:
        if type(impa) == str:
            imp = impa
        else:
            imp = impa[0]
        module = pool.moduleProviding(imp)
        for interface in module['interfaces']:
            if interface['name'] == imp:
                for mname in interface['methods']:
                    method = interface['methods'][mname]
                    paramStrA = ''
                    bodyCode = body_code_from_name(method['name'], component)
                    if p_utils.communication_is_ice(impa):
                        for p in method['params']:
                            # delim
                            if paramStrA == '':
                                delim = ''
                            else:
                                delim = ', '
                            # decorator
                            ampersand = '&'
                            if p['decorator'] == 'out':
                                const = ''
                            else:
                                if component.language.lower() == "cpp":
                                    const = 'const '
                                else:
                                    const = ''
                                    ampersand = ''
                                if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
                                    ampersand = ''
                            # STR
                            paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
                        result += method['return'] + ' SpecificWorker::' + interface['name'] + "_" + method[
                            'name'] + '(' + paramStrA + ")\n{\n//implementCODE\n" + bodyCode + "\n}\n\n"
                    else:
                        paramStrA = module['name'] + "ROS::" + method['name'] + "::Request &req, " + module[
                            'name'] + "ROS::" + method['name'] + "::Response &res"
                        if imp in component.iceInterfaces:
                            result += 'bool SpecificWorker::ROS' + method[
                                'name'] + '(' + paramStrA + ")\n{\n//implementCODE\n" + bodyCode + "\n}\n\n"
                        else:
                            result += 'bool SpecificWorker::' + method[
                                'name'] + '(' + paramStrA + ")\n{\n//implementCODE\n" + bodyCode + "\n}\n\n"
    return result

def subscribes(component):
    result = ""
    pool = component.idsl_pool
    ros_types = pool.getRosTypes()
    for impa in component.subscribesTo:
        if type(impa) == str:
            imp = impa
        else:
            imp = impa[0]
        module = pool.moduleProviding(imp)
        if module == None:
            raise ValueError('\nCan\'t find module providing %s\n' % imp)
        for interface in module['interfaces']:
            if interface['name'] == imp:
                for mname in interface['methods']:
                    method = interface['methods'][mname]
                    paramStrA = ''
                    bodyCode = body_code_from_name(method['name'], component)
                    if p_utils.communication_is_ice(impa):
                        for p in method['params']:
                            # delim
                            if paramStrA == '':
                                delim = ''
                            else:
                                delim = ', '
                            # decorator
                            ampersand = '&'
                            if p['decorator'] == 'out':
                                const = ''
                            else:
                                if component.language.lower() == "cpp":
                                    const = 'const '
                                else:
                                    const = ''
                                    ampersand = ''
                                if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
                                    ampersand = ''
                            # STR
                            paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
                        result += "//SUBSCRIPTION to " + method['name'] + " method from " + interface[
                            'name'] + " interface\n"
                        result += method['return'] + ' SpecificWorker::' + interface['name'] + "_" + method[
                            'name'] + '(' + paramStrA + ")\n{\n//subscribesToCODE\n" + bodyCode + "\n}\n\n"
                    else:
                        for p in method['params']:
                            # delim
                            if paramStrA == '':
                                delim = ''
                            else:
                                delim = ', '
                            # decorator
                            ampersand = '&'
                            if p['decorator'] == 'out':
                                const = ''
                            else:
                                const = 'const '
                                ampersand = ''
                            if p['type'] in ('float', 'int'):
                                p['type'] = "std_msgs::" + p['type'].capitalize() + "32"
                            elif p['type'] in ('uint8', 'uint16', 'uint32', 'uint64'):
                                p['type'] = "std_msgs::UInt" + p['type'].split('t')[1]
                            elif p['type'] in ros_types:
                                p['type'] = "std_msgs::" + p['type'].capitalize()
                            elif not '::' in p['type']:
                                p['type'] = module['name'] + "ROS::" + p['type']
                            # STR
                            paramStrA += delim + p['type'] + ' ' + p['name']
                        if imp in component.iceInterfaces:
                            result += 'void SpecificWorker::ROS' + method[
                                'name'] + '(' + paramStrA + ")\n{\n//subscribesToCODE\n" + bodyCode + "\n}\n\n"
                        else:
                            result += 'void SpecificWorker::' + method[
                                'name'] + '(' + paramStrA + ")\n{\n//subscribesToCODE\n" + bodyCode + "\n}\n\n"
    return result

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
bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
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

def agm_specific_code(component):
    result = ""
    if ('agmagent' in [x.lower() for x in component.options]) and (
            'innermodelviewer' in [x.lower() for x in component.options]):
        result += REGENERATE_INNERMODEL

    if 'agmagent' in [x.lower() for x in component.options]:
        result += SET_PARAMETERS_AND_POSSIBLE_ACTIVATION
        agentName = component.name
        if component.language.lower() == "cpp":
            proxy = "agmexecutive_proxy"
        else:
            proxy = "*agmexecutive_proxy.get()"
        result += Template(SEND_MODIFICATION_PROPOSAL).substitute(proxy=proxy, agent_name=agentName)
    return result

def proxy_map_type(component):
    if component.language.lower() == 'cpp':
        return "MapPrx&"
    else:
        return "TuplePrx"

def proxy_map_name(component):
    if component.language.lower() == 'cpp':
        return "mprx"
    else:
        return "tprx"

def statemachine_finalize_emit(statemachine):
    result = ""
    if statemachine is not None and statemachine['machine']['default']:
        result += "emit t_compute_to_finalize();\n"
    return result

def state_machine_start(statemachine):
    result = ""
    if statemachine is not None:
        result += statemachine['machine']['name'] + ".start();\n"
    return result

def statemachine_initialize_to_compute(statemachine):
    result = ""
    if statemachine is not None and statemachine['machine']['default']:
        result += "emit this->t_initialize_to_compute();\n"
    return result

def get_template_dict(component):
    return {
        'year': str(datetime.date.today().year),
        'proxy_map_type': proxy_map_type(component),
        'proxy_map_name': proxy_map_name(component),
        'innermodelviewer_code': innermodelviewer_code(component.innermodelviewer),
        'agmagent_attributes': agmagent_attributes(component),
        'statemachine_finalize_emit': statemachine_finalize_emit(component.statemachine),
        'innermodel_and_viewer_attribute_init': innermodel_and_viewer_attribute_init(component.innermodelviewer),
        'agm_innermodel_association': agm_innermodel_association(component),
        'state_machine_start': state_machine_start(component.statemachine),
        'statemachine_initialize_to_compute': statemachine_initialize_to_compute(component.statemachine),
        'compute_method': compute_method(component, component.statemachine),
        'statemachine_methods_creation': statemachine_methods_creation(component.statemachine),
        'implements': implements(component),
        'subscribes': subscribes(component),
        'agm_specific_code': agm_specific_code(component)
    }