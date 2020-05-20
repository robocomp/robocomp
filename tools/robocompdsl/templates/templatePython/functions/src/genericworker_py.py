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
            codStateMachine = ""
            codQState = ""
            codQStateParallel = ""
            codQFinalState = ""
            Machine = statemachine['machine']['name']
            codStateMachine = "self." + Machine + "= QtCore.QStateMachine()"

            if statemachine['machine']['contents']['states'] is not None:
                for state in statemachine['machine']['contents']['states']:
                    aux = "self." + state + "_state = QtCore.QState(self." + Machine + ")\n"
                    if statemachine['substates'] is not None:
                        for substates in statemachine['substates']:
                            if state == substates['parent']:
                                if substates['parallel'] is "parallel":
                                    aux = "self." + state + "_state = QtCore.QState(QtCore.QState.ParallelStates, self." + Machine + ")\n"
                                    break
                    if "ParallelStates" in aux:
                        codQStateParallel += aux
                    else:
                        codQState += aux
            if statemachine['machine']['contents']['initialstate'] is not None:
                state = statemachine['machine']['contents']['initialstate']
                aux = "self." + state + "_state = QtCore.QState(self." + Machine + ")\n"
                if statemachine['substates'] is not None:
                    for substates in statemachine['substates']:
                        if state == substates['parent']:
                            if substates['parallel'] is "parallel":
                                aux = "self." + state + "_state = QtCore.QState(QtCore.QState.ParallelStates,self." + Machine + ")\n"
                                break
                if "ParallelStates" in aux:
                    codQStateParallel += aux
                else:
                    codQState += aux
            if statemachine['machine']['contents']['finalstate'] is not None:
                state = statemachine['machine']['contents']['finalstate']
                codQFinalState += "self." + state + "_state = QtCore.QFinalState(self." + Machine + ")\n"
            result += "#State Machine\n"
            result += codStateMachine + '\n'
            result += codQState + '\n'
            result += codQFinalState + '\n'
            result += codQStateParallel + '\n'
            codStateMachine = ""
            codQState = ""
            codQStateParallel = ""
            codQFinalState = ""
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
                                codQStateParallel += aux
                            else:
                                codQState += aux
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
                            codQStateParallel += aux
                        else:
                            codQState += aux
                    if substates['contents']['finalstate'] is not None:
                        codQFinalState += "self." + substates['contents'][
                            'finalstate'] + "_state = QtCore.QFinalState(self." + substates['parent'] + "_state)\n"
                    result += codStateMachine + '\n'
                    result += codQState + '\n'
                    result += codQFinalState + '\n'
                    result += codQStateParallel + '\n'
                    codStateMachine = ""
                    codQState = ""
                    codQStateParallel = ""
                    codQFinalState = ""
            result += "#------------------\n"

            codaddTransition = ""
            codaddState = ""
            codConnect = ""
            codsetInitialState = ""
            if statemachine['machine']['contents']['transitions'] is not None:
                for transi in statemachine['machine']['contents']['transitions']:
                    for dest in transi['dests']:
                        codaddTransition += "self." + transi['src'] + "_state.addTransition(self.t_" + \
                                            transi['src'] + "_to_" + dest + ", self." + dest + "_state)\n"
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['transitions'] is not None:
                        for transi in substates['contents']['transitions']:
                            for dest in transi['dests']:
                                codaddTransition += "self." + transi[
                                    'src'] + "_state.addTransition(self.t_" + transi[
                                                        'src'] + "_to_" + dest + ", self." + dest + "_state)\n"
            if statemachine['machine']['contents']['states'] is not None:
                for state in statemachine['machine']['contents']['states']:
                    codConnect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
            if statemachine['machine']['contents']['initialstate'] is not None:
                state = statemachine['machine']['contents']['initialstate']
                codsetInitialState += "self." + statemachine['machine'][
                    'name'] + ".setInitialState(self." + state + "_state)\n"
                codConnect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
            if statemachine['machine']['contents']['finalstate'] is not None:
                state = statemachine['machine']['contents']['finalstate']
                codConnect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    if substates['contents']['initialstate'] is not None:
                        state = substates['contents']['initialstate']
                        codsetInitialState += "self." + substates[
                            'parent'] + "_state.setInitialState(self." + state + "_state)\n"
                        codConnect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
                    if substates['contents']['finalstate'] is not None:
                        state = substates['contents']['finalstate']
                        codConnect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
                    if substates['contents']['states'] is not None:
                        for state in substates['contents']['states']:
                            codConnect += "self." + state + "_state.entered.connect(self.sm_" + state + ")\n"
            if statemachine['machine']['default']:
                codConnect += "self.timer.timeout.connect(self.t_compute_to_compute)\n"
            result += "#Initialization State machine\n"
            result += codaddTransition + '\n'
            result += codaddState + '\n'
            result += codConnect + '\n'
            result += codsetInitialState + '\n'
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
        if self.component.usingROS == True:
            # CREANDO CLASES PARA LOS PUBLISHERS
            for imp in self.component.publishes:
                nname = imp
                while type(nname) != type(''):
                    nname = nname[0]
                module = pool.moduleProviding(nname)
                if module == None:
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
                nname = imp
                while type(nname) != type(''):
                    nname = nname[0]
                module = pool.moduleProviding(nname)
                if module == None:
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
                                paramStrA = ''
                                for p in method['params']:
                                    # delim
                                    if paramStrA == '': paramStrA = p['name']
                                result += "<TABHERE>def " + mname + "(self, " + paramStrA + "):\n"
                                result += "<TABHERE><TABHERE>return self.srv_" + mname + "(" + paramStrA + ")\n"
        return result

    # TODO: Refactor, too much repeated code.
    def ros_imports(self):
        result = ""
        pool = self.component.idsl_pool
        if self.component.usingROS == True:
            result += 'import rospy\n'
            result += 'from std_msgs.msg import *\n'
            for iface in self.component.publishes + self.component.subscribesTo:
                if type(iface) == str:
                    iface_name = iface
                else:
                    iface_name = iface[0]
                if not communication_is_ice(iface):
                    module = pool.moduleProviding(iface_name)
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
                    module = pool.moduleProviding(iface_name)
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
            result += 'from ' + module['name'] + ' import *\n'
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