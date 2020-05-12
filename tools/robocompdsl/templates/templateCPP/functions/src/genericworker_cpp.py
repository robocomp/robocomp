import datetime

from dsl_parsers.parsing_utils import get_name_number, communication_is_ice


def statemachine_initialization(statemachine, visual):
    result = ""
    if statemachine is not None:
        codaddTransition = ""
        codaddState = ""
        codConnect = ""
        codsetInitialState = ""
        states = ""

        if statemachine['machine']['contents']['states'] is not None:
            for state in statemachine['machine']['contents']['states']:
                childMode = "QState::ExclusiveStates"
                if statemachine['substates'] is not None:
                    for substates in statemachine['substates']:
                        if state == substates['parent']:
                            if substates['parallel'] is "parallel":
                                childMode = "QState::ParallelStates"
                                break
                if not visual:
                    codaddState += state + "State = new QState(" + childMode + ");\n"
                    codaddState += statemachine['machine']['name'] + ".addState(" + state + "State);\n"
                else:
                    codaddState += state + "State = " + statemachine['machine'][
                        'name'] + ".addState(\"" + state + "\"," + childMode + ");\n"

                codConnect += "QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
                states += state + ","

        if statemachine['machine']['contents']['initialstate'] is not None:
            state = statemachine['machine']['contents']['initialstate']
            childMode = "QState::ExclusiveStates"
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if state == substates['parent']:
                        if substates['parallel'] is "parallel":
                            childMode = "QState::ParallelStates"
                            break
            if not visual:
                codaddState += state + "State = new QState(" + childMode + ");\n"
                codaddState += statemachine['machine']['name'] + ".addState(" + state + "State);\n"
            else:
                codaddState += state + "State = " + statemachine['machine'][
                    'name'] + ".addState(\"" + state + "\"," + childMode + ");\n"
            codsetInitialState += statemachine['machine']['name'] + ".setInitialState(" + state + "State);\n"
            codConnect += "QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
            states += state + ","

        if statemachine['machine']['contents']['finalstate'] is not None:
            state = statemachine['machine']['contents']['finalstate']
            if not visual:
                codaddState += state + "State = new QFinalState();\n"
                codaddState += statemachine['machine']['name'] + ".addState(" + state + "State);\n"
            else:
                codaddState += state + "State = " + statemachine['machine'][
                    'name'] + ".addFinalState(\"" + state + "\");\n"
            codConnect += "QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
            states += state + ","

        if statemachine['substates'] is not None:
            for substates in statemachine['substates']:
                childMode = "QState::ExclusiveStates"
                if substates['contents']['states'] is not None:
                    for state in substates['contents']['states']:
                        for sub in statemachine['substates']:
                            if state == sub['parent']:
                                if sub['parallel'] is "parallel":
                                    childMode = "QState::ParallelStates"
                                    break
                        if not visual:
                            codaddState += state + "State = new QState(" + childMode + ", " + substates[
                                'parent'] + "State);\n"
                            codaddState += statemachine['machine']['name'] + ".addState(" + state + "State);\n"
                        else:
                            codaddState += state + "State = " + statemachine['machine'][
                                'name'] + ".addState(\"" + state + "\", " + childMode + ", " + substates[
                                               'parent'] + "State);\n"

                if substates['contents']['initialstate'] is not None:
                    childMode = "QState::ExclusiveStates"
                    for sub in statemachine['substates']:
                        if substates['contents']['initialstate'] == sub['parent']:
                            if sub['parallel'] is "parallel":
                                childMode = "QState::ParallelStates"
                                break
                    if not visual:
                        codaddState += substates['contents'][
                            'initialstate'] + "State = new QState(" + childMode + ", " + substates[
                                           'parent'] + "State);\n"
                        codaddState += statemachine['machine']['name'] + ".addState(" + state + "State);\n"
                    else:
                        codaddState += substates['contents']['initialstate'] + "State = " + statemachine['machine'][
                            'name'] + ".addState(\"" + state + "\", " + childMode + ", " + substates[
                                           'parent'] + "State);\n"

                if substates['contents']['finalstate'] is not None:
                    if not visual:
                        codaddState += substates['contents']['finalstate'] + "State = new QFinalState(" + \
                                       substates['parent'] + "State);\n"
                    else:
                        codaddState += substates['contents']['finalstate'] + "State = " + statemachine['machine'][
                            'name'] + ".addFinalState(\"" + state + "\");\n"

        if statemachine['machine']['contents']['transitions'] is not None:
            for transi in statemachine['machine']['contents']['transitions']:
                for dest in transi['dests']:
                    if not visual:
                        codaddTransition += transi['src'] + "State->addTransition(" + "this, SIGNAL(t_" + \
                                            transi['src'] + "_to_" + dest + "()), " + dest + "State);\n"
                    else:
                        codaddTransition += statemachine['machine']['name'] + ".addTransition(" + transi[
                            'src'] + "State, this, SIGNAL(t_" + transi[
                                                'src'] + "_to_" + dest + "()), " + dest + "State);\n"
        if statemachine['substates'] is not None:
            for substates in statemachine['substates']:
                if substates['contents']['transitions'] is not None:
                    for transi in substates['contents']['transitions']:
                        for dest in transi['dests']:
                            if not visual:
                                codaddTransition += transi[
                                    'src'] + "State->addTransition(" + "this, SIGNAL(t_" + transi[
                                                        'src'] + "_to_" + dest + "()), " + dest + "State);\n"
                            else:
                                codaddTransition += statemachine['machine']['name'] + ".addTransition(" + transi[
                                    'src'] + "State, this, SIGNAL(t_" + transi[
                                                        'src'] + "_to_" + dest + "()), " + dest + "State);\n"

        if statemachine['substates'] is not None:
            for substates in statemachine['substates']:
                if substates['contents']['initialstate'] is not None:
                    state = substates['contents']['initialstate']
                    codsetInitialState += substates[
                        'parent'] + "State->setInitialState(" + state + "State);\n"
                    codConnect += "QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
                    states += state + ","
                if substates['contents']['finalstate'] is not None:
                    state = substates['contents']['finalstate']
                    codConnect += "QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
                    states += state + ","
                if substates['contents']['states'] is not None:
                    for state in substates['contents']['states']:
                        codConnect += "QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
                        states += state + ","
        if statemachine['machine']['default']:
            codConnect += "QObject::connect(&timer, SIGNAL(timeout()), this, SIGNAL(t_compute_to_compute()));\n"
        result += "//Initialization State machine\n"
        result += codaddState + "\n"
        result += codsetInitialState + "\n"
        result += codaddTransition + "\n"
        result += codConnect + "\n"
        result += "//------------------\n"
    return result

def require_and_publish_proxies_creation(component):
    result = ""
    cont = 0
    for iface, num in get_name_number(component.requires):
        if communication_is_ice(iface):
            name = iface[0]
            if component.language.lower() == 'cpp':
                result += name.lower() + num + "_proxy = (*(" + name + "Prx*)mprx[\"" + name + "Proxy" + num + "\"]);\n"
            else:
                result += name.lower() + num + "_proxy = std::get<" + str(cont) + ">(tprx);\n"
        cont = cont + 1

    for iface, num in get_name_number(component.publishes):
        if communication_is_ice(iface):
            name = iface[0]
            if component.language.lower() == 'cpp':
                result += name.lower() + num + "_pubproxy = (*(" + name + "Prx*)mprx[\"" + name + "Pub" + num + "\"]);\n"
            else:
                result += name.lower() + num + "_pubproxy = std::get<" + str(cont) + ">(tprx);\n"
        cont = cont + 1
    return result

def ros_nodes_creation(component):
    result = ""
    if component.usingROS == True:
        # INICIALIZANDO SUBSCRIBERS
        pool = component.idsl_pool
        for iface in component.subscribesTo:
            module = pool.moduleProviding(iface.name)
            if module == None:
                raise ValueError('\nCan\'t find module providing %s \n' % iface.name)
            if not communication_is_ice(iface):
                for interface in module['interfaces']:
                    if interface['name'] == iface.name:
                        for mname in interface['methods']:
                            s = "\"" + mname + "\""
                            if iface.name in component.iceInterfaces:
                                result += iface.name + "_" + mname + " = node.subscribe(" + s + ", 1000, &GenericWorker::ROS" + mname + ", this);\n"
                            else:
                                result += iface.name + "_" + mname + " = node.subscribe(" + s + ", 1000, &GenericWorker::" + mname + ", this);\n"
        # INICIALIZANDO IMPLEMENTS
        for iface in component.implements:
            module = pool.moduleProviding(iface.name)
            if module == None:
                raise ('\nCan\'t find module providing %s\n' % iface.name)
            if not communication_is_ice(iface):
                for interface in module['interfaces']:
                    if interface['name'] == iface.name:
                        for mname in interface['methods']:
                            s = "\"" + mname + "\""
                            if iface.name in component.iceInterfaces:
                                result += iface.name + "_" + mname + " = node.advertiseService(" + s + ", &GenericWorker::ROS" + mname + ", this);\n"
                            else:
                                result += iface.name + "_" + mname + " = node.advertiseService(" + s + ", &GenericWorker::" + mname + ", this);\n"
    return result

def ros_proxies_creation(component):
    result = ""
    for publish in component.publishes:
        pubs = publish
        while type(pubs) != type(''):
            pubs = pubs[0]
        if not communication_is_ice(publish):
            if pubs in component.iceInterfaces:
                result += pubs.lower() + "_rosproxy = new Publisher" + pubs + "(&node);\n"
            else:
                result += pubs.lower() + "_proxy = new Publisher" + pubs + "(&node);\n"
    for require in component.requires:
        req = require
        while type(req) != type(''):
            req = req[0]
        if not communication_is_ice(require):
            if req in component.iceInterfaces:
                result += req.lower() + "_rosproxy = new ServiceClient" + req + "(&node);\n"
            else:
                result += req.lower() + "_proxy = new ServiceClient" + req + "(&node);\n"
    return result

GUI_SETUP_STR = """
#ifdef USE_QTGUI
	setupUi(this);
	show();
#endif
"""
def gui_setup(component_gui):
    result = ""
    if component_gui is not None:
        result += GUI_SETUP_STR
    return result

AGM_CREATEACTION_STR = """
RoboCompPlanning::Action GenericWorker::createAction(std::string s)
{
	// Remove useless characters
	char chars[]="()";
		for (unsigned int i=0; i<strlen(chars); ++i)
	{
		s.erase(std::remove(s.begin(), s.end(), chars[i]), s.end());
	}

		// Initialize string parsing
	RoboCompPlanning::Action ret;
	istringstream iss(s);

	// Get action (first segment)
	if (not iss)
	{
		printf("agent %s: received invalid action (%s) -> (%d)\\n", PROGRAM_NAME, __FILE__, __LINE__);
		exit(-1);
	}
	else
	{
		iss >> ret.name;
	}

	do
	{
		std::string ss;
		iss >> ss;
		ret.symbols.push_back(ss);
	} while (iss);

	return ret;
}
"""

AGM_ACTIVATE = """
bool GenericWorker::activate(const BehaviorParameters &prs)
{
	printf("Worker::activate\\n");
	mutex->lock();
	p = prs;
	active = true;
	iter = 0;
	mutex->unlock();
	return active;
}
"""

AGM_DEACTIVATE = """
bool GenericWorker::deactivate()
{
	printf("Worker::deactivate\\n");
	mutex->lock();
	active = false;
	iter = 0;
	mutex->unlock();
	return active;
}
"""

AGM_SETPARAMETERSANDPOSSIBLEACTIVATION_STR = """
bool GenericWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		// Action
		p.action = createAction(params["action"].value);

		// Fill received plan
		p.plan.clear();
		QStringList actionList = QString::fromStdString(params["plan"].value).split(QRegExp("[()]+"), QString::SkipEmptyParts);
		for (int32_t actionString=0; actionString<actionList.size(); actionString++)
		{
			std::vector<string> elementsVec;
			QStringList elements = actionList[actionString].remove(QChar('\\n')).split(QRegExp("\\\\s+"), QString::SkipEmptyParts);
			for (int32_t elem=0; elem<elements.size(); elem++)
			{
				elementsVec.push_back(elements[elem].toStdString());
			}
			p.plan.push_back(elementsVec);
		}
	}
	catch (...)
	{
		return false;
	}

	// Check if we should reactivate the component
	if (isActive())
	{
		activate(p);
		reactivated = true;
	}

	return true;
}
"""

def agm_methods(component):
    result = ""
    try:
        # TODO: move to component method (component.is_agm())
        if 'agmagent' in [x.lower() for x in component.options]:
            result += AGM_CREATEACTION_STR
            result += AGM_ACTIVATE
            result += AGM_DEACTIVATE
            result += AGM_SETPARAMETERSANDPOSSIBLEACTIVATION_STR
    except:
        #TODO: fix it
        pass
    return result

def constructor_proxies(component):
    result = ""
    if component.language.lower() == 'cpp':
        result += "MapPrx& mprx"
    else:
        result += "TuplePrx tprx"
    return result

def inherited_constructor(component):
    if component.gui:
        return "Ui_guiDlg()"
    else:
        return "QObject()"

def compute_connect(component):
    result = ""
    if component.statemachine_path is None:
        result += "connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));\n"
    return result

def get_template_dict(component):
    return {
        'year': str(datetime.date.today().year),
        'constructor_proxies': constructor_proxies(component),
        'inherited_constructor': inherited_constructor(component),
        'statemachine_initialization': statemachine_initialization(component.statemachine, component.statemachine_visual),
        'require_and_publish_proxies_creation': require_and_publish_proxies_creation(component),
        'ros_nodes_creation': ros_nodes_creation(component),
        'ros_proxies_creation': ros_proxies_creation(component),
        'gui_setup': gui_setup(component.gui),
        'compute_connect': compute_connect(component),
        'agm_methods': agm_methods(component)


    }
