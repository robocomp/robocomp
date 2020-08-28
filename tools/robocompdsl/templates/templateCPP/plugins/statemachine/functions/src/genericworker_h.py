import datetime
from string import Template

from templates.common.templatedict import TemplateDict




class genericworker_h(TemplateDict):

    def __init__(self, component):
        super(genericworker_h, self).__init__()
        self.component = component
        self['statemachine_includes'] = self.statemachine_includes()
        self['statemachine_creation'] = self.statemachine_creation()
        self['statemachine_slots'] = self.statemachine_slots()
        self['statemachine_signals'] = self.statemachine_signals()

    def statemachine_includes(self):
        result = ""
        if self.component.statemachine is not None:
            result += "#include <QStateMachine>\n"
            result += "#include <QState>\n"
            if self.component.statemachine_visual:
                result += "#include \"statemachinewidget/qstateMachineWrapper.h\"\n"
        return result

    def statemachine_creation(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            code_qstates = ""
            lsstates = ""
            if not self.component.statemachine_visual:
                cod_qstate_machine = "QStateMachine " + statemachine['machine']['name'] + ";\n"
            else:
                cod_qstate_machine = "QStateMachineWrapper " + statemachine['machine']['name'] + ";\n"
            if statemachine['machine']['contents']['states'] is not None:
                for state in statemachine['machine']['contents']['states']:
                    aux = "QState *" + state + "State;\n"
                    lsstates += state + ","
                    if statemachine['substates'] is not None:
                        for substates in statemachine['substates']:
                            if state == substates['parent']:
                                if substates['parallel'] is "parallel":
                                    aux = "QState *" + state + "State;\n"
                                    break
                    code_qstates += aux
            if statemachine['machine']['contents']['initialstate'] is not None:
                state = statemachine['machine']['contents']['initialstate']
                aux = "QState *" + state + "State;\n"
                lsstates += state + ","
                if statemachine['substates'] is not None:
                    for substates in statemachine['substates']:
                        if state == substates['parent']:
                            if substates['parallel'] is "parallel":
                                aux = "QState *" + state + "State;\n"
                                break
                code_qstates += aux

            if statemachine['machine']['contents']['finalstate'] is not None:
                state = statemachine['machine']['contents']['finalstate']
                code_qstates += "QFinalState *" + state + "State;\n"
                lsstates += state + ","

            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['states'] is not None:
                        for state in substates['contents']['states']:
                            aux = "QState *" + state + "State;\n"
                            lsstates += state + ","
                            for sub in statemachine['substates']:
                                if state == sub['parent']:
                                    if sub['parallel'] is "parallel":
                                        aux = "QState *" + state + "State;\n"
                                        break
                            code_qstates += aux
                    if substates['contents']['initialstate'] is not None:
                        aux = "QState *" + substates['contents']['initialstate'] + "State;\n"
                        lsstates += state + ","
                        for sub in statemachine['substates']:
                            if state == sub['parent']:
                                if sub['parallel'] is "parallel":
                                    aux = "QState *" + state + "State;\n"
                                    break
                        code_qstates += aux
                    if substates['contents']['finalstate'] is not None:
                        code_qstates += "QFinalState *" + substates['contents']['finalstate'] + "State;\n"
                        lsstates += state + ","

            result += "//State Machine\n"
            result += cod_qstate_machine+"\n"
            result += code_qstates+"\n"
            result += "//-------------------------\n"
        return result

    # TODO: Refactor for submachines
    def statemachine_slots(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            sm_virtual_methods = ""
            if statemachine['machine']['contents']['states'] is not None:
                for state in statemachine['machine']['contents']['states']:
                    sm_virtual_methods += "virtual void sm_" + state + "() = 0;\n"
            if statemachine['machine']['contents']['initialstate'] is not None:
                sm_virtual_methods += "virtual void sm_" + statemachine['machine']['contents']['initialstate'] + "() = 0;\n"
            if statemachine['machine']['contents']['finalstate'] is not None:
                sm_virtual_methods += "virtual void sm_" + statemachine['machine']['contents']['finalstate'] + "() = 0;\n"
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['states'] is not None:
                        for state in substates['contents']['states']:
                            sm_virtual_methods += "virtual void sm_" + state + "() = 0;\n"
                    if substates['contents']['initialstate'] is not None:
                        sm_virtual_methods += "virtual void sm_" + substates['contents'][
                            'initialstate'] + "() = 0;\n"
                    if substates['contents']['finalstate'] is not None:
                        sm_virtual_methods += "virtual void sm_" + substates['contents'][
                            'finalstate'] + "() = 0;\n"
            result += "//Slots funtion State Machine\n"
            result += sm_virtual_methods + '\n'
            result += "//-------------------------\n"
        return result

    def statemachine_signals(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            codsignals = ""
            if statemachine['machine']['contents']['transitions'] is not None:
                for transi in statemachine['machine']['contents']['transitions']:
                    for dest in transi['dests']:
                        codsignals += "void t_" + transi['src'] + "_to_" + dest + "();\n"
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['transitions'] is not None:
                        for transi in substates['contents']['transitions']:
                            for dest in transi['dests']:
                                codsignals += "void t_" + transi['src'] + "_to_" + dest + "();\n"
            result += "//Signals for State Machine\n"
            result += codsignals + '\n'
            result += "//-------------------------\n"
        return result

