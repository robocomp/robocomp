import datetime
from string import Template

from robocompdsl.templates.common.templatedict import TemplateDict

DSR_SLOTS_STR = """\

# =============== DSR SLOTS  ================
# =============================================

def update_node_att(self, id: int, attribute_names: [str]):
    console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

def update_node(self, id: int, type: str):
    console.print(f"UPDATE NODE: {id} {type}", style='green')

def delete_node(self, id: int):
    console.print(f"DELETE NODE:: {id} ", style='green')

def update_edge(self, fr: int, to: int, type: str):

    console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
    console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

def delete_edge(self, fr: int, to: int, type: str):
    console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
"""


DSR_IMPORT_STR = """\
from pydsr import *

"""

DSR_INIT_STR = """\

# YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
self.agent_id = "_CHANGE_THIS_ID_"
self.g = DSRGraph(0, "pythonAgent", self.agent_id)

try:
    signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
    signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
    signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
    signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
    signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
    signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
    console.print("signals connected")
except RuntimeError as e:
    print(e)

"""

class src_specificworker_py(TemplateDict):
    def __init__(self, component):
        super(TemplateDict, self).__init__()
        self.component = component
        self['dsr_slots'] = self.dsr_slots()
        self['dsr_import'] = self.dsr_import()
        self['dsr_init'] = self.dsr_init()

    def dsr_slots(self):
        result = ""
        if self.component.dsr:
            result += DSR_SLOTS_STR
        return result

    def dsr_import(self):
        result = ""
        if self.component.dsr:
            result += DSR_IMPORT_STR
        return result

    def dsr_init(self):
        result = ""
        if self.component.dsr:
            result += DSR_INIT_STR
        return result
