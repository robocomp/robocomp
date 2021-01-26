import datetime
from string import Template

from templates.common.templatedict import TemplateDict

DSR_SLOTS_STR = """\

def update_node_att(id: int, attribute_names: [str]):
    print("UPDATE NODE ATT: ", id, " ", attribute_names)

def update_node(id: int, type: str):
    print("UPDATE NODE: ", id," ",  type)

def delete_node(id: int):
    print("DELETE NODE: ", id)

def update_edge(fr: int, to: int, type : str):
    print("UPDATE EDGE: ", fr," ", to," ", type)

def update_edge_att(fr: int, to: int, attribute_names : [str]):
    print("UPDATE EDGE ATT: ", fr," ", to," ", attribute_names)

def delete_edge(fr: int, to: int, type : str):
    print("DELETE EDGE: ", fr," ", to," ", type)
"""


DSR_IMPORT_STR = """\
from pydsr import *

"""

DSR_INIT_STR = """\

self.g = DSRGraph(0, "pythonAgent", 111)

try:
    signals.connect(self.g, signals.UPDATE_NODE_ATTR, update_node_att)
    signals.connect(self.g, signals.UPDATE_NODE, update_node)
    signals.connect(self.g, signals.DELETE_NODE, delete_node)
    signals.connect(self.g, signals.UPDATE_EDGE, update_edge)
    signals.connect(self.g, signals.UPDATE_EDGE_ATTR, update_edge_att)
    signals.connect(self.g, signals.DELETE_EDGE, delete_edge)
    print("signals connected")
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