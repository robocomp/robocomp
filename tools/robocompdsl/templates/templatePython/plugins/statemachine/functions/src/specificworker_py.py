import datetime
from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number
from templates.templatePython.plugins.base.functions import function_utils as utils
from templates.common.templatedict import TemplateDict





STATEMACHINE_INITIALIZE_METHOD_STR = """
#
# sm_${state_method}
#
@QtCore.Slot()
def sm_${state_method}(self):
    print("Entered state ${state_method}")
    self.t_initialize_to_compute.emit()
    pass
    
"""

STATEMACHINE_COMPUTE_METHOD_STR = """
#
# sm_${state_method}
#
@QtCore.Slot()
def sm_${state_method}(self):
    print("Entered state ${state_method}")
    self.compute()
    pass

"""

STATEMACHINE_METHOD_STR = """
#
# sm_${state_method}
#
@QtCore.Slot()
def sm_${state_method}(self):
    print("Entered state ${state_method}")
    pass
"""

class src_specificworker_py(TemplateDict):
    def __init__(self, component):
        super(TemplateDict, self).__init__()
        self.component = component
        self['statemachine_start_and_destroy'] = self.statemachine_start_and_destroy()
        self['statemachine_slots'] = self.statemachine_slots()

    # TODO: Refactor main states and substates
    def statemachine_slots(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            cod_virtuals = ""

            # Generate code for the methods of the StateMachine.
            if statemachine['machine']['contents']['initialstate'] is not None:

                # TODO: code to add comments with the transitions
                # if sm['machine']['contents']['transitions'] is not None:
                #     for transi in sm['machine']['contents']['transitions']:
                #             if sm['machine']['contents']['initialstate'] == trasi["src"]
                #                 codsignals += "<TABHERE>#<TABHERE>%s >>> %s" % ( transi['src'], sm['machine']['contents']['initialstate'])
                # if sm['machine']['contents']['transitions'] is not None:
                #     for transi in sm['machine']['contents']['transitions']:
                #         for dest in transi['dests']:
                #             if  sm['machine']['contents']['initialstate'] == dest
                #                 codsignals += "<TABHERE>#<TABHERE>%s <<< %s"%(sm['machine']['contents']['initialstate'], transi['src'])

                if statemachine['machine']['default']:
                    cod_virtuals += Template(STATEMACHINE_INITIALIZE_METHOD_STR).substitute(state_method=statemachine['machine']['contents']['initialstate'])
                else:
                    cod_virtuals += Template(STATEMACHINE_METHOD_STR).substitute(state_method=statemachine['machine']['contents']['initialstate'])

            if statemachine['machine']['contents']['states'] is not None:
                for state in sorted(statemachine['machine']['contents']['states']):
                    if statemachine['machine']['default']:
                        if state == 'compute':
                            cod_virtuals += Template(STATEMACHINE_COMPUTE_METHOD_STR).substitute(state_method=state)
                    else:
                        cod_virtuals += Template(STATEMACHINE_METHOD_STR).substitute(state_method=state)

            if statemachine['machine']['contents']['finalstate'] is not None:
                cod_virtuals += Template(STATEMACHINE_METHOD_STR).substitute(state_method=statemachine['machine']['contents']['finalstate'])

            # Generate code for the methods of the StateMachine transitions for substates.
            if statemachine['substates'] is not None:
                for substates in statemachine['substates']:
                    # TODO: Add commented header with the parent of this methods.
                    if substates['contents']['initialstate'] is not None:
                        cod_virtuals += Template(STATEMACHINE_METHOD_STR).substitute(state_method=substates['contents']['initialstate'])
                    if substates['contents']['states'] is not None:
                        for state in sorted(substates['contents']['states']):
                            cod_virtuals += Template(STATEMACHINE_METHOD_STR).substitute(state_method=state)
                    if substates['contents']['finalstate'] is not None:
                        cod_virtuals += Template(STATEMACHINE_METHOD_STR).substitute(state_method=substates['contents']['finalstate'])

            result += "# =============== Slots methods for State Machine ===================\n"
            result += "# ===================================================================\n"
            result += cod_virtuals+'\n'
            result += "# =================================================================\n"
            result += "# =================================================================\n"
        return result


    def statemachine_start_and_destroy(self):
        result = ""
        statemachine = self.component.statemachine
        if statemachine is not None:
            result += "self." + statemachine['machine']['name'] + ".start()\n"
            if statemachine['machine']['default']:
                result += "self.destroyed.connect(self.t_compute_to_finalize)\n"
        return result
