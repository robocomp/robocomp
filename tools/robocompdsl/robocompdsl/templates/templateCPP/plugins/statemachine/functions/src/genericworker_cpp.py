from string import Template
from templates.common.templatedict import TemplateDict




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



class genericworker_cpp(TemplateDict):
    def __init__(self, component):
        super(genericworker_cpp, self).__init__()
        self.component = component
        self['statemachine_initialization'] = self.statemachine_initialization()

    @staticmethod
    def _statemachine_state_is_parallel(state, substates):
        if substates is not None:
            for substates in substates:
                if state == substates['parent']:
                    if substates['parallel'] == "parallel":
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
