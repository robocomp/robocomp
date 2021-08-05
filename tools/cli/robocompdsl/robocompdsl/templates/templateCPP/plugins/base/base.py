import os
import importlib
import glob
from robocompdsl.templates.common.plugin_collection import Plugin

class Base(Plugin):
    def __init__(self):
        super(Base, self).__init__()
        self.abs_path = os.path.abspath(os.path.dirname(__file__))
        self.path = os.path.relpath(os.path.dirname(__file__))
        self.classes = {}
        self.load_functions()
