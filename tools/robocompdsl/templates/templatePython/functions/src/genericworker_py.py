import datetime
from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number

SLICE_LOAD_STR = """\
Ice.loadSlice("-I ./src/ --all ./src/${interface_name}.ice")
"""

ROS_MSG_IMPORT_STR = """
try:
    from ${module_name}ROS.msg import *
except:
    print(\"Couldn\'t load msg\")
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
        self['ros_imports'] = self.ros_imports()
        self['ui_import'] = self.ui_import()
        self['ros_class_creation'] = self.ros_class_creation()
        self['qt_class_type'] = self.qt_class_type()
        self['statemachine_signals'] = self.statemachine_signals()
        self['requires_proxies'] = self.requires_proxies()
        self['publishes_proxies'] = self.publishes_proxies()
        self['gui_setup'] = self.gui_setup()
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

    def implements_and_subscribes_imports(self):
        result = ""
        for im in self.component.implements + self.component.subscribesTo:
            if communication_is_ice(im):
                result += 'from ' + im.name.lower() + 'I import *\n'
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

    # TODO: extract code strings and refactor
    def ros_class_creation(self):
        result = ""
        pool = self.component.idsl_pool
        if self.component.usingROS:
            # CREANDO CLASES PARA LOS PUBLISHERS
            for imp in self.component.publishes:
                nname = imp.name
                module = pool.module_providing_interface(nname)
                if module is None:
                    raise ValueError('\nCan\'t find module providing %s\n' % nname)
                if not communication_is_ice(imp):
                    result += "#class for rosPublisher\n"
                    result += "class Publisher" + nname + "():\n"
                    result += "<TABHERE>def __init__(self):\n"
                    for interface in module['interfaces']:
                        if interface['name'] == nname:
                            for mname in interface['methods']:
                                method = interface['methods'][mname]
                                for p in method['params']:
                                    s = "\"" + mname + "\""
                                    if p['type'] in ('float', 'int'):
                                        result += "<TABHERE><TABHERE>self.pub_" + mname + " = rospy.Publisher(" + s + ", " + \
                                                  p['type'].capitalize() + "32, queue_size=1000)\n"
                                    elif p['type'] in ('uint8', 'uint16', 'uint32', 'uint64'):
                                        result += "<TABHERE><TABHERE>self.pub_" + mname + " = rospy.Publisher(" + s + ", UInt" + \
                                                  p['type'].split('t')[1] + ", queue_size=1000)\n"
                                    elif p['type'] in IDSLPool.getRosTypes():
                                        result += "<TABHERE><TABHERE>self.pub_" + mname + " = rospy.Publisher(" + s + ", " + \
                                                  p['type'].capitalize() + ", queue_size=1000)\n"
                                    elif '::' in p['type']:
                                        result += "<TABHERE><TABHERE>self.pub_" + mname + " = rospy.Publisher(" + s + ", " + \
                                                  p['type'].split('::')[1] + ", queue_size=1000)\n"
                                    else:
                                        result += "<TABHERE><TABHERE>self.pub_" + mname + " = rospy.Publisher(" + s + ", " + \
                                                  p['type'] + ", queue_size=1000)\n"
                    for interface in module['interfaces']:
                        if interface['name'] == nname:
                            for mname in interface['methods']:
                                method = interface['methods'][mname]
                                for p in method['params']:
                                    result += "<TABHERE>def " + mname + "(self, " + p['name'] + "):\n"
                                    result += "<TABHERE><TABHERE>self.pub_" + mname + ".publish(" + p['name'] + ")\n"
            # CREANDO CLASES PARA LOS REQUIRES
            for imp in self.component.requires:
                nname = imp.name
                module = pool.module_providing_interface(nname)
                if module is None:
                    raise ValueError('\nCan\'t find module providing %s\n' % nname)
                if not communication_is_ice(imp):
                    result += "#class for rosServiceClient\n"
                    result += "class ServiceClient" + nname + "():\n"
                    result += "<TABHERE>def __init__(self):\n"
                    for interface in module['interfaces']:
                        if interface['name'] == nname:
                            for mname in interface['methods']:
                                method = interface['methods'][mname]  # for p in method['params']:
                                s = "\"" + mname + "\""
                                result += "<TABHERE><TABHERE>self.srv_" + mname + " = rospy.ServiceProxy(" + s + ", " + mname + ")\n"
                    for interface in module['interfaces']:
                        if interface['name'] == nname:
                            for mname in interface['methods']:
                                method = interface['methods'][mname]
                                param_str_a = ''
                                for p in method['params']:
                                    # delim
                                    if param_str_a == '': param_str_a = p['name']
                                result += "<TABHERE>def " + mname + "(self, " + param_str_a + "):\n"
                                result += "<TABHERE><TABHERE>return self.srv_" + mname + "(" + param_str_a + ")\n"
        return result

    # TODO: Refactor, too much repeated code.
    def ros_imports(self):
        result = ""
        pool = self.component.idsl_pool
        if self.component.usingROS:
            result += 'import rospy\n'
            result += 'from std_msgs.msg import *\n'
            for iface in self.component.publishes + self.component.subscribesTo:
                if type(iface) == str:
                    iface_name = iface
                else:
                    iface_name = iface[0]
                if not communication_is_ice(iface):
                    module = pool.module_providing_interface(iface_name)
                    for interface in module['interfaces']:
                        if interface['name'] == iface_name:
                            for mname in interface['methods']:
                                result += Template(ROS_MSG_IMPORT_STR).substitute(module_name=module['name'])
            for iface in self.component.requires + self.component.implements:
                if type(iface) == str:
                    iface_name = iface
                else:
                    iface_name = iface[0]
                if not communication_is_ice(iface):
                    module = pool.module_providing_interface(iface_name)
                    for interface in module['interfaces']:
                        if interface['name'] == iface_name:
                            for mname in interface['methods']:
                                result += 'from ' + module['name'] + 'ROS.srv import *'
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
                if rq in self.component.iceInterfaces:
                    result += "self." + rq.lower() + "_rosproxy = ServiceClient" + rq + "()\n"
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
                if pub in self.component.iceInterfaces:
                    result += "self." + pub.lower() + "_rosproxy = Publisher" + pub + "()\n"
                else:
                    result += "self." + pub.lower() + "_proxy = Publisher" + pub + "()\n"
        return result
