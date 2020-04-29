from string import Template

import dsl_parsers.parsing_utils as p_utils


def body_code_from_name(name, component):
    bodyCode = ""
    if p_utils.is_agm1_agent(component):
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
            bodyCode = "\tQMutexLocker lockIM(mutex);\n <TABHERE>AGMModelConverter::fromIceToInternal(w, worldModel);\n \n<TABHERE>innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel));"
            if 'innermodelviewer' in [x.lower() for x in component.options]:
                bodyCode += "\n<TABHERE>regenerateInnerModelViewer();"
        elif name == 'selfEdgeAdded':
            bodyCode = "\tQMutexLocker lockIM(mutex);\n <TABHERE>try { worldModel->addEdgeByIdentifiers(nodeid, nodeid, edgeType, attributes); } catch(...){ printf(\"Couldn't add an edge. Duplicate?\\n\"); }\n \n<TABHERE>try { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf(\"Can't extract an InnerModel from the current model.\\n\"); }"
            if 'innermodelviewer' in [x.lower() for x in component.options]:
                bodyCode += "\n<TABHERE>regenerateInnerModelViewer();"
        elif name == 'selfEdgeDeleted':
            bodyCode = "\tQMutexLocker lockIM(mutex);\n <TABHERE>try { worldModel->removeEdgeByIdentifiers(nodeid, nodeid, edgeType); } catch(...) { printf(\"Couldn't remove an edge\\n\"); }\n \n<TABHERE>try { innerModel = std::make_shared<InnerModel>(AGMInner::extractInnerModel(worldModel)); } catch(...) { printf(\"Can't extract an InnerModel from the current model.\\n\"); }"
            if 'innermodelviewer' in [x.lower() for x in component.options]:
                bodyCode += "\n<TABHERE>regenerateInnerModelViewer();"
        #######################################
        # code to implement AGMCommonBehavior #
        #######################################
        elif name == 'activateAgent':
            bodyCode = "<TABHERE>bool activated = false;\n<TABHERE>if (setParametersAndPossibleActivation(prs, activated))\n<TABHERE>{\n<TABHERE><TABHERE>if (not activated)\n<TABHERE><TABHERE>{\n<TABHERE><TABHERE><TABHERE>return activate(p);\n<TABHERE><TABHERE>}\n<TABHERE>}\n<TABHERE>else\n<TABHERE>{\n<TABHERE><TABHERE>return false;\n<TABHERE>}\n<TABHERE>return true;"
        elif name == 'deactivateAgent':
            bodyCode = "<TABHERE>return deactivate();"
        elif name == 'getAgentState':
            bodyCode = "<TABHERE>StateStruct s;\n<TABHERE>if (isActive())\n<TABHERE>{\n<TABHERE><TABHERE>s.state = RoboCompAGMCommonBehavior::StateEnum::Running;\n<TABHERE>}\n<TABHERE>else\n<TABHERE>{\n<TABHERE><TABHERE>s.state = RoboCompAGMCommonBehavior::StateEnum::Stopped;\n<TABHERE>}\n<TABHERE>s.info = p.action.name;\n<TABHERE>return s;"
        elif name == 'getAgentParameters':
            bodyCode = "<TABHERE>return params;"
        elif name == 'setAgentParameters':
            bodyCode = "<TABHERE>bool activated = false;\n<TABHERE>return setParametersAndPossibleActivation(prs, activated);"
        elif name == 'uptimeAgent':
            bodyCode = "<TABHERE>return 0;"
        elif name == 'reloadConfigAgent':
            bodyCode = "<TABHERE>return true;"

    elif p_utils.is_agm2_agent(component):
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
        if p_utils.is_agm1_agent(component) or p_utils.is_agm2_agent(component):
            result += "<TABHERE>active = false;\n"
            result += "<TABHERE>worldModel = AGMModel::SPtr(new AGMModel());\n"
            result += "<TABHERE>worldModel->name = " + "\"worldModel\";\n"

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

def agm_innermodel_association(component):
    result = ""
    try:
        if p_utils.is_agm1_agent(component):
            result += "<TABHERE>innerModel = std::make_shared<InnerModel>(new InnerModel());\n"
            result += "<TABHERE>try\n"
            result += "<TABHERE>{\n"
            result += "<TABHERE><TABHERE>RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();\n"
            result += "<TABHERE><TABHERE>AGMExecutiveTopic_structuralChange(w);\n"
            result += "<TABHERE>}\n"
            result += "<TABHERE>catch(...)\n"
            result += "<TABHERE>{\n"
            result += "<TABHERE><TABHERE>printf(\"The executive is probably not running, waiting for first AGM model publication...\");\n"
            result += "<TABHERE>}\n"
        elif p_utils.is_agm2_agent(component):
            result += "// TODO: Here we should ask the DSR for the current model for initialization purposes.\n"
    except:
        pass
    return result


def compute_method(component, statemachine):
    result = ""
    if (statemachine is not None and statemachine['machine']['default'] is True) or component.statemachine is None:
        result += "void SpecificWorker::compute()\n"
        result += "{\n"
        result += "//computeCODE\n"
        result += "//QMutexLocker locker(mutex);\n"
        result += "//<TABHERE>try\n"
        result += "//<TABHERE>{\n"
        result += "//<TABHERE><TABHERE>camera_proxy->getYImage(0,img, cState, bState);\n"
        result += "//<TABHERE><TABHERE>memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));\n"
        result += "//<TABHERE><TABHERE>searchTags(image_gray);\n"
        result += "//<TABHERE>}\n"
        result += "//<TABHERE>catch(const Ice::Exception &e)\n"
        result += "//<TABHERE>{\n"
        result += "//<TABHERE><TABHERE>std::cout << \"Error reading from Camera\" << e << std::endl;\n"
        result += "//<TABHERE>}\n"
        if component.usingROS == True:
            result += "<TABHERE>ros::spinOnce();\n"
        if component.innermodelviewer:
            result += "#ifdef USE_QTGUI\n"
            result += "<TABHERE>if (innerModelViewer) innerModelViewer->update();\n"
            result += "<TABHERE>osgView->frame();\n"
            result += "#endif\n"
        result += "}\n"
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

def implements(component, pool):
    result = ""
    if 'implements' in component:
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

def subscribes(component, pool):
    result = ""
    ros_types = pool.getRosTypes()
    if 'subscribesTo' in component:
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
