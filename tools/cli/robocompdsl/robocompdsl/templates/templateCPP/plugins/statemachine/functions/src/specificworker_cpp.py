from string import Template
from robocompdsl.templates.common.templatedict import TemplateDict




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

class specificworker_cpp(TemplateDict):
    def __init__(self, component):
        super(specificworker_cpp, self).__init__()
        self.component = component
        self['statemachine_finalize_emit'] = self.statemachine_finalize_emit()
        self['state_machine_start'] = self.state_machine_start()
        self['statemachine_initialize_to_compute'] = self.statemachine_initialize_to_compute()
        self['statemachine_methods_creation'] = self.statemachine_methods_creation()

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