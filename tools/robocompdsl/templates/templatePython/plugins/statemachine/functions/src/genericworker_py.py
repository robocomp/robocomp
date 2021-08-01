from string import Template
from templates.common.templatedict import TemplateDict



STATEMACHINE_SLOT_STR = """
@QtCore.Slot()
def sm_${state_name}(self):
    print(\"Error: lack sm_${state_name} in Specificworker\")
    sys.exit(-1)
"""


class src_genericworker_py(TemplateDict):
    def __init__(self, component):
        super(src_genericworker_py, self).__init__()
        self.component = component
        self['statemachine_signals'] = self.statemachine_signals()
        self['statemachine_states_creation'] = self.statemachine_states_creation()
        self['statemachine_slots_creation'] = self.statemachine_slots_creation()

    # TODO: refactor
    def statemachine_slots_creation(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            codVirtuals = ""
            codcompsubclas = ""
            if statemachine['machine']['contents']['states'] is not None:
                for state in statemachine['machine']['contents']['states']:
                    codVirtuals += Template(STATEMACHINE_SLOT_STR).substitute(state_name=state)
            if statemachine['machine']['contents']['initialstate'] is not None:
                codVirtuals += Template(STATEMACHINE_SLOT_STR).substitute(
                    state_name=statemachine['machine']['contents']['initialstate'])
            if statemachine['machine']['contents']['finalstate'] is not None:
                codVirtuals += Template(STATEMACHINE_SLOT_STR).substitute(
                    state_name=statemachine['machine']['contents']['finalstate'])
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['states'] is not None:
                        for state in substates['contents']['states']:
                            codVirtuals += Template(STATEMACHINE_SLOT_STR).substitute(state_name=state)
                    if substates['contents']['initialstate'] is not None:
                        codVirtuals += Template(STATEMACHINE_SLOT_STR).substitute(
                            state_name=substates['contents']['initialstate'])
                    if substates['contents']['finalstate'] is not None:
                        codVirtuals += Template(STATEMACHINE_SLOT_STR).substitute(
                            state_name=substates['contents']['finalstate'])
                result += "#Slots funtion State Machine\n"
                result += codVirtuals + '\n'
                result += "#-------------------------\n"
        return result


    # TODO: Refactooooor
    def statemachine_states_creation(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            machine = statemachine['machine']['name']
            code_qstates = ""
            code_parallel_qstate = ""
            code_final_qstate = ""
            code_state_machine = "self." + machine + "= QtCore.QStateMachine()"

            if statemachine['machine']['contents']['states'] is not None:
                for state in statemachine['machine']['contents']['states']:
                    aux = "self." + state + "_state = QtCore.QState(self." + machine + ")\n"
                    if statemachine['substates'] is not None:
                        for substates in statemachine['substates']:
                            if state == substates['parent']:
                                if substates['parallel'] == "parallel":
                                    aux = "self." + state + "_state = QtCore.QState(QtCore.QState.ParallelStates, self." + machine + ")\n"
                                    break
                    if "ParallelStates" in aux:
                        code_parallel_qstate += aux
                    else:
                        code_qstates += aux
            if statemachine['machine']['contents']['initialstate'] is not None:
                state = statemachine['machine']['contents']['initialstate']
                aux = "self." + state + "_state = QtCore.QState(self." + machine + ")\n"
                if statemachine['substates'] is not None:
                    for substates in statemachine['substates']:
                        if state == substates['parent']:
                            if substates['parallel'] == "parallel":
                                aux = "self." + state + "_state = QtCore.QState(QtCore.QState.ParallelStates,self." + machine + ")\n"
                                break
                if "ParallelStates" in aux:
                    code_parallel_qstate += aux
                else:
                    code_qstates += aux
            if statemachine['machine']['contents']['finalstate'] is not None:
                state = statemachine['machine']['contents']['finalstate']
                code_final_qstate += "self." + state + "_state = QtCore.QFinalState(self." + machine + ")\n"
            result += "#State Machine\n"
            result += code_state_machine + '\n'
            result += code_qstates + '\n'
            result += code_final_qstate + '\n'
            result += code_parallel_qstate + '\n'
            code_state_machine = ""
            code_qstates = ""
            code_parallel_qstate = ""
            code_final_qstate = ""
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['states'] is not None:
                        for state in substates['contents']['states']:
                            aux = "self." + state + "_state = QtCore.QState(self." + substates[
                                'parent'] + "_state)\n"
                            for sub in statemachine['substates']:
                                if state == sub['parent']:
                                    if sub['parallel'] == "parallel":
                                        aux = "self." + state + "_state = QtCore.QState(QtCore.QState.ParallelStates, self." + \
                                              substates['parent'] + "_state)\n"
                                        break
                            if "ParallelStates" in aux:
                                code_parallel_qstate += aux
                            else:
                                code_qstates += aux
                    if substates['contents']['initialstate'] is not None:
                        aux = "self." + substates['contents'][
                            'initialstate'] + "_state = QtCore.QState(self." + substates['parent'] + "_state)\n"
                        for sub in statemachine['substates']:
                            if state == sub['parent']:
                                if sub['parallel'] == "parallel":
                                    aux = "self." + state + "_state = QtCore.QState(QtCore.QState.ParallelStates, self." + \
                                          substates['parent'] + "_state)\n"
                                    break
                        if "ParallelStates" in aux:
                            code_parallel_qstate += aux
                        else:
                            code_qstates += aux
                    if substates['contents']['finalstate'] is not None:
                        code_final_qstate += "self." + substates['contents'][
                            'finalstate'] + "_state = QtCore.QFinalState(self." + substates['parent'] + "_state)\n"
                    result += code_state_machine + '\n'
                    result += code_qstates + '\n'
                    result += code_final_qstate + '\n'
                    result += code_parallel_qstate + '\n'
                    code_state_machine = ""
                    code_qstates = ""
                    code_parallel_qstate = ""
                    code_final_qstate = ""
            result += "#------------------\n"

            code_add_transition = ""
            code_add_state = ""
            code_connect = ""
            code_set_initial_state = ""
            if statemachine['machine']['contents']['transitions'] is not None:
                for transi in statemachine['machine']['contents']['transitions']:
                    for dest in transi['dests']:
                        code_add_transition += "self." + transi['src'] + "_state.addTransition(self.t_" + \
                                            transi['src'] + "_to_" + dest + ", self." + dest + "_state)\n"
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['transitions'] is not None:
                        for transi in substates['contents']['transitions']:
                            for dest in transi['dests']:
                                code_add_transition += "self." + transi[
                                    'src'] + "_state.addTransition(self.t_" + transi[
                                                        'src'] + "_to_" + dest + ", self." + dest + "_state)\n"
            if statemachine['machine']['contents']['states'] is not None:
                for state in statemachine['machine']['contents']['states']:
                    code_connect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
            if statemachine['machine']['contents']['initialstate'] is not None:
                state = statemachine['machine']['contents']['initialstate']
                code_set_initial_state += "self." + statemachine['machine'][
                    'name'] + ".setInitialState(self." + state + "_state)\n"
                code_connect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
            if statemachine['machine']['contents']['finalstate'] is not None:
                state = statemachine['machine']['contents']['finalstate']
                code_connect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['initialstate'] is not None:
                        state = substates['contents']['initialstate']
                        code_set_initial_state += "self." + substates[
                            'parent'] + "_state.setInitialState(self." + state + "_state)\n"
                        code_connect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
                    if substates['contents']['finalstate'] is not None:
                        state = substates['contents']['finalstate']
                        code_connect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
                    if substates['contents']['states'] is not None:
                        for state in substates['contents']['states']:
                            code_connect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
            if statemachine['machine']['default']:
                code_connect += "self.timer.timeout.connect(self.t_compute_to_compute)\n"
            result += "#Initialization State machine\n"
            result += code_add_transition + '\n'
            result += code_add_state + '\n'
            result += code_connect + '\n'
            result += code_set_initial_state + '\n'
            result += "#------------------\n"
        return result


    def statemachine_signals(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            codsignals = ""
            if statemachine['machine']['contents']['transitions'] is not None:
                for transi in statemachine['machine']['contents']['transitions']:
                    for dest in transi['dests']:
                        codsignals += "t_" + transi['src'] + "_to_" + dest + " = QtCore.Signal()\n"
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['transitions'] is not None:
                        for transi in substates['contents']['transitions']:
                            for dest in transi['dests']:
                                codsignals += "t_" + transi['src'] + "_to_" + dest + " = QtCore.Signal()\n"
            result += "#Signals for State Machine\n"
            result += codsignals + '\n'
            result += "#-------------------------\n"
        return result
