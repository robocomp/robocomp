import datetime
from string import Template

import dsl_parsers.parsing_utils as p_utils
from templates.templateCPP.plugins.base.functions import function_utils as utils
from templates.common.templatedict import TemplateDict

INNERMODEL_COMPUTE_STR = """\
#ifdef USE_QTGUI
    if (innerModelViewer) innerModelViewer->update();
    osgView->frame();
#endif
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

class specificworker_cpp(TemplateDict):
    def __init__(self, component):
        super(specificworker_cpp, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['proxy_map_type'] = self.proxy_map_type()
        self['proxy_map_name'] = self.proxy_map_name()
        self['compute_method'] = self.compute_method()
        self['implements'] = self.implements()
        self['subscribes'] = self.subscribes()
        self['interface_specific_comment'] = self.interface_specific_comment()


    # TODO: Extract pieces of code to strings and refactor
    def body_code_from_name(self, name):
        body_code = ""
        if self.component.is_agm_agent():
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
        return body_code



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