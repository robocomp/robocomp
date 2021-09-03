import os
from string import Template
import importlib
from robocompdsl.templates.common.plugin_collection import Plugin
from ....common.abstracttemplatesmanager import CustomTemplate as CTemplate

class StateMachine(Plugin):
    def __init__(self):
        super(StateMachine, self).__init__()
        self.abs_path = os.path.abspath(os.path.dirname(__file__))
        self.path = os.path.relpath(os.path.dirname(__file__))
        self.load_functions()
