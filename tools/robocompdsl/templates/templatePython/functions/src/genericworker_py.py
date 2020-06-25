import datetime
from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number
from .. import function_utils as utils

SLICE_LOAD_STR = """\
Ice.loadSlice("-I ./src/ --all ./src/${interface_name}.ice")
"""

LIST_CLASSES_STR = """\
class ${list_type}(list):
    def __init__(self, iterable=list()):
        super(${list_type}, self).__init__(iterable)

    def append(self, item):
        assert isinstance(item, ${item_type})
        super(${list_type}, self).append(item)

    def extend(self, iterable):
        for item in iterable:
            assert isinstance(item, ${item_type})
        super(${list_type}, self).extend(iterable)

    def insert(self, index, item):
        assert isinstance(item, ${item_type})
        super(${list_type}, self).insert(index, item)

setattr(${module_name}, "${list_type}", ${list_type})

"""



GUI_IMPORT_STR = """
try:
    from ui_mainUI import *
except:
    print("Can\'t import UI file. Did you run \'make\'?")
    sys.exit(-1)
"""

GUI_SETUP_STR = """
self.ui = Ui_guiDlg()
self.ui.setupUi(self)
self.show()
"""

STATEMACHINE_SLOT_STR = """
@QtCore.Slot()
def sm_${state_name}(self):
    print(\"Error: lack sm_${state_name} in Specificworker\")
    sys.exit(-1)
"""


class TemplateDict(dict):
    def __init__(self, component):
        super(TemplateDict, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['load_slice_and_create_imports'] = self.load_slice_and_create_imports()
        self['implements_and_subscribes_imports'] = self.implements_and_subscribes_imports()
        self['ui_import'] = self.ui_import()
        self['qt_class_type'] = self.qt_class_type()
        self['statemachine_signals'] = self.statemachine_signals()
        self['requires_proxies'] = self.requires_proxies()
        self['publishes_proxies'] = self.publishes_proxies()
        self['gui_setup'] = self.gui_setup()
        self['statemachine_states_creation'] = self.statemachine_states_creation()
        self['statemachine_slots_creation'] = self.statemachine_slots_creation()
        self['create_lists_classes'] = self.create_lists_classes()

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

    def implements_and_subscribes_imports(self):
        result = ""
        for im in self.component.implements + self.component.subscribesTo:
            if communication_is_ice(im):
                result += 'import ' + im.name.lower() + 'I\n'
        return result

    def qt_class_type(self):
        if self.component.gui is not None:
            inherit_from = 'QtWidgets.' + self.component.gui[1]
        else:
            inherit_from = 'QtCore.QObject'
        return inherit_from

    def gui_setup(self):
        result = ""
        if self.component.gui is not None:
            result += GUI_SETUP_STR
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
                                if substates['parallel'] is "parallel":
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
                            if substates['parallel'] is "parallel":
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
                                    if sub['parallel'] is "parallel":
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
                                if sub['parallel'] is "parallel":
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

    def ui_import(self):
        result = ""
        if self.component.gui is not None:
            result += GUI_IMPORT_STR
        return result


    # TODO: Check if can be merged with SERVANT_PY.py slice_loading function
    def load_slice_and_create_imports(self, includeDirectories=None):
        result = ""
        import os
        for imp in sorted(set(self.component.recursiveImports + self.component.imports)):
            file_name = os.path.basename(imp)
            name = os.path.splitext(file_name)[0]
            result += Template(SLICE_LOAD_STR).substitute(interface_name=name)
            module = DSLFactory().from_file(file_name, includeDirectories=includeDirectories)
            result += f"import {module['name']}\n"
        return result



    def create_lists_classes(self):
        result = ""
        for idsl in sorted(set(self.component.recursiveImports + self.component.imports)):
            module = self.component.idsl_pool.module_providing_interface(idsl.split('.')[0])
            for sequence in module['sequences']:
                item_type = utils.get_type_string(sequence['typeSequence'], module['name'])
                if item_type == 'bytes': continue
                result += Template(LIST_CLASSES_STR).substitute(list_type=sequence['name'].split('/')[1],
                                                                item_type=item_type,
                                                                module_name=module['name'])
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

    # TODO: Refactor this and publishes with a zip?
    def requires_proxies(self):
        result = ""
        for req, num in get_name_number(self.component.requires):
            if isinstance(req, str):
                rq = req
            else:
                rq = req[0]
            if communication_is_ice(req):
                result += "self." + rq.lower() + num + "_proxy = mprx[\"" + rq + "Proxy" + num + "\"]\n"
            else:
                result += "self." + rq.lower() + "_proxy = ServiceClient" + rq + "()\n"
        return result

    def publishes_proxies(self):
        result = ""
        for pb, num in get_name_number(self.component.publishes):
            if isinstance(pb, str):
                pub = pb
            else:
                pub = pb[0]
            if communication_is_ice(pb):
                result += "self." + pub.lower() + num + "_proxy = mprx[\"" + pub + "Pub" + num + "\"]\n"
            else:
                result += "self." + pub.lower() + "_proxy = Publisher" + pub + "()\n"
        return result
