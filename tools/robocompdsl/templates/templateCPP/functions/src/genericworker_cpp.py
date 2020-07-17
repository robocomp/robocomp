import datetime
from string import Template
from dsl_parsers.parsing_utils import get_name_number, communication_is_ice


GUI_SETUP_STR = """
#ifdef USE_QTGUI
	setupUi(this);
	show();
#endif
"""


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
bool GenericWorker::setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated)
{
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	for (RoboCompAGMCommonBehavior::ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
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

STATEMACHINE_STATE_CREATION = """\
${state_name}State = new ${state_type}(${child_mode}${parent});
"""

STATEMACHINE_STATE_ADD = """\
${statemachine_name}.addState(${state_name}State);
"""

VISUAL_STATEMACHINE_STATE_CREATION = """\
${state_name}State = ${statemachine_name}.${add_state_method}("${state_name}",${child_mode}${parent});
"""

STATEMACHINE_TRANSITION_CREATION = """\
${state_name}State = new ${state_type}(${child_mode});
${statemachine_name}.addState(${state_name}State);
"""

VISUAL_STATEMACHINE_TRANSITION_CREATION = """\
${state_name}State = new ${state_type}(${child_mode});
${statemachine_name}.addState(${state_name}State);
"""

CPP_TYPES = ['int', 'float', 'bool', 'void']

class TemplateDict(dict):
    def __init__(self, component):
        super(TemplateDict, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['constructor_proxies'] = self.constructor_proxies()
        self['inherited_constructor'] = self.inherited_constructor()
        self['statemachine_initialization'] = self.statemachine_initialization()
        self['require_and_publish_proxies_creation'] = self.require_and_publish_proxies_creation()
        self['gui_setup'] = self.gui_setup()
        self['compute_connect'] = self.compute_connect()
        self['agm_methods'] = self.agm_methods()

    @staticmethod
    def _statemachine_state_is_parallel(state, substates):
        if substates is not None:
            for substates in substates:
                if state == substates['parent']:
                    if substates['parallel'] is "parallel":
                        return True
        return False

    @staticmethod
    def _statemachine_state_creation(statemachine_name, state, parent="", visual=False, is_parallel=False, is_final=False):
        states_str = ""
        connects_str = ""
        child_mode = "QState::ExclusiveStates"
        if is_parallel:
            child_mode = "QState::ParallelStates"
        if is_final:
            add_state_method = "addFinalState"
            state_type = "QFinalState"
            child_mode = ""
            if parent:
                parent = "%sState" % parent
        else:
            if parent:
                parent = ", %sState" % parent
            add_state_method = "addFinalState"
            state_type = "QState"

        if not visual:
            states_str += Template(STATEMACHINE_STATE_CREATION).substitute(state_name=state,
                                                                           child_mode=child_mode,
                                                                           statemachine_name=statemachine_name,
                                                                           state_type=state_type,
                                                                           parent=parent)
            if not parent:
                states_str += Template(STATEMACHINE_STATE_ADD).substitute(state_name=state,
                                                                          statemachine_name=statemachine_name)

        else:
            states_str += Template(VISUAL_STATEMACHINE_STATE_CREATION).substitute(state_name=state,
                                                                                  child_mode=child_mode,
                                                                                  statemachine_name=statemachine_name,
                                                                                  add_state_method=add_state_method,
                                                                                  parent=parent)
        connects_str += "QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
        return states_str, connects_str

    def _statemachine_states_creation(self, statemachine_name, machine, substates=None, visual=False, is_sub=False):
        code_add_states = ""
        code_connects = ""
        code_set_initial_state = ""
        contents = machine['contents']
        parent = machine['parent'] if 'parent' in machine else ""

        # Code for initial state
        if contents['initialstate'] is not None:
            state = contents['initialstate']
            states_str, connects_str = self._statemachine_state_creation(statemachine_name,
                                                                         state=state,
                                                                         parent=parent,
                                                                         visual=visual)

            code_add_states += states_str
            code_connects += connects_str
            if not is_sub:
                code_set_initial_state += statemachine_name + ".setInitialState(" + state + "State);\n"
            else:
                code_set_initial_state += machine['parent'] + "State->setInitialState(" + state + "State);\n"

        # Code for states
        if contents['states'] is not None:
            for state in contents['states']:
                is_parallel = self._statemachine_state_is_parallel(state, substates)
                states_str, connects_str = self._statemachine_state_creation(statemachine_name,
                                                                             state=state,
                                                                             parent=parent,
                                                                             visual=visual,
                                                                             is_parallel=is_parallel)
                code_add_states += states_str
                code_connects += connects_str

        # Code for final state
        if contents['finalstate'] is not None:
            state = contents['finalstate']
            states_str, connects_str = self._statemachine_state_creation(statemachine_name,
                                                                         state=state,
                                                                         parent=parent,
                                                                         visual=visual,
                                                                         is_final=True)
            code_add_states += states_str
            code_connects += connects_str

        return code_add_states, code_connects, code_set_initial_state

    @staticmethod
    def _statemachine_transitions_creation(statemachine_name, machine, visual):
        code_add_transitions = ""
        if machine['contents']['transitions'] is not None:
            for transition in machine['contents']['transitions']:
                for dest in transition['dests']:
                    if not visual:
                        code_add_transitions += transition['src'] + "State->addTransition(" + "this, SIGNAL(t_" + \
                                            transition['src'] + "_to_" + dest + "()), " + dest + "State);\n"
                    else:
                        code_add_transitions += statemachine_name + ".addTransition(" + transition[
                            'src'] + "State, this, SIGNAL(t_" + transition[
                                                'src'] + "_to_" + dest + "()), " + dest + "State);\n"
        return code_add_transitions

    def statemachine_initialization(self):
        result = ""
        statemachine = self.component.statemachine
        visual = self.component.statemachine_visual
        if statemachine is not None:
            code_add_states, code_connects, code_set_initial_states = self._statemachine_states_creation(
                statemachine['machine']['name'],
                statemachine['machine'],
                statemachine['substates'],
                visual)

            if statemachine['substates'] is not None:
                for substate in statemachine['substates']:
                    states, connects, initials = self._statemachine_states_creation(
                        statemachine['machine']['name'],
                        substate,
                        None,
                        visual,
                        is_sub=True)
                    code_add_states += states
                    code_connects += connects
                    code_set_initial_states += initials

            code_add_transitions = ""
            if statemachine['machine']['contents']['transitions'] is not None:
                code_add_transitions = self._statemachine_transitions_creation(
                    statemachine['machine']['name'],
                    statemachine['machine'],
                    visual)
            if statemachine['substates'] is not None:
                for substate in statemachine['substates']:
                    code_add_transitions += self._statemachine_transitions_creation(
                        statemachine['machine']['name'],
                        substate,
                        visual)

            if statemachine['machine']['default']:
                code_connects += "QObject::connect(&timer, SIGNAL(timeout()), this, SIGNAL(t_compute_to_compute()));\n"
            result += "//Initialization State machine\n"
            result += code_add_states + "\n"
            result += code_set_initial_states + "\n"
            result += code_add_transitions + "\n"
            result += code_connects + "\n"
            result += "//------------------\n"
        return result

    def require_and_publish_proxies_creation(self):
        result = ""
        cont = 0
        for interface, num in get_name_number(self.component.requires):
            if communication_is_ice(interface):
                name = interface.name
                if self.component.language.lower() == 'cpp':
                    prx_type = name
                    if prx_type not in CPP_TYPES and '::' not in prx_type:
                        module = self.component.idsl_pool.module_providing_interface(name)
                        prx_type = f"{module['name']}::{prx_type}"
                    result += name.lower() + num + "_proxy = (*(" + prx_type + "Prx*)mprx[\"" + name + "Proxy" + num + "\"]);\n"
                else:
                    result += name.lower() + num + "_proxy = std::get<" + str(cont) + ">(tprx);\n"
            cont = cont + 1

        for interface, num in get_name_number(self.component.publishes):
            if communication_is_ice(interface):
                name = interface.name
                prx_type = name
                if prx_type not in CPP_TYPES and '::' not in prx_type:
                    module = self.component.idsl_pool.module_providing_interface(name)
                    prx_type = f"{module['name']}::{prx_type}"
                if self.component.language.lower() == 'cpp':
                    result += name.lower() + num + "_pubproxy = (*(" + prx_type + "Prx*)mprx[\"" + name + "Pub" + num + "\"]);\n"
                else:
                    result += name.lower() + num + "_pubproxy = std::get<" + str(cont) + ">(tprx);\n"
            cont = cont + 1
        return result


    def gui_setup(self):
        result = ""
        if self.component.gui is not None:
            result += GUI_SETUP_STR
        return result

    def agm_methods(self):
        result = ""
        # TODO: move to component method (self.component.is_agm())
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += AGM_CREATEACTION_STR
            result += AGM_ACTIVATE
            result += AGM_DEACTIVATE
            result += AGM_SETPARAMETERSANDPOSSIBLEACTIVATION_STR
        return result

    def constructor_proxies(self):
        result = ""
        if self.component.language.lower() == 'cpp':
            result += "MapPrx& mprx"
        else:
            result += "TuplePrx tprx"
        return result

    def inherited_constructor(self):
        if self.component.gui:
            return "Ui_guiDlg()"
        else:
            return "QObject()"

    def compute_connect(self):
        result = ""
        if self.component.statemachine_path is None:
            result += "connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));\n"
        return result
