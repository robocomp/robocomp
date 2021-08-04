import os
import importlib
import glob
from templates.common.plugin_collection import Plugin

class StateMachine(Plugin):
    def __init__(self):
        super(StateMachine, self).__init__()
        self.abs_path = os.path.abspath(os.path.dirname(__file__))
        self.path = os.path.relpath(os.path.dirname(__file__))
        self.classes = {}
        self.load_functions()
