#!/usr/bin/env python3
import os
import sys
from workspace import Workspace

sys.path.append('/opt/robocomp/python')

class rccd:
        def __init__(self):
                self.ws = Workspace()

        def find_component_path(self, searched_component):
                path = self.ws.find_only_component(searched_component)
                if (path):
                        print(path)
                else:
                        pass

        def list_comp(self):
                components=self.ws.list_components_names()
                for component in components:
                        print(component)

if __name__ == '__main__':
        rccd = rccd()
        args = sys.argv[1]
        if args == "list":
                rccd.list_comp()
        else:
                try:
                        rccd.find_component_path(args)
                except KeyboardInterrupt:
                        pass
