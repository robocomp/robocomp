#!/usr/bin/python

from __future__ import print_function
import os
import sys
import argparse
from workspace import workspace as WS

def main():
    parser = argparse.ArgumentParser(description="configures and build components ")
    parser.add_argument('component', nargs='?', help='name of the component to build if omitted curent workspace is build')
    args = parser.parse_args()

    if not args.component:
        cpath = os.path.abspath('.')
        wspath = WS.find_workspace(cpath)
        if os.path.exists(".rc_workspace"):
            os.chdir("./build")
            os.system("cmake ../src")
        elif wspath:
            rest = cpath[len(wspath):].split('/')
            # if len(rest) >= 3:
            #     componentPath = os.path.join(wspath,rest[1],rest[2])
            #     os.chdir(wspath+"/build")
            #     os.system("cmake " + componentPath)
            # else:
            os.chdir(wspath+"/build")
            os.system("cmake ../src")
        else:
            parser.error("This is not a valid robocomp workspace")
    else:
        component = args.component
        paths = WS.find_component_src(component)
        if len(paths) == 0:
            parser.error("No such component exists")
        for path in paths:
            print("Invoking cmake at {0}".format(path))
            os.chdir(path)
            if not os.path.exists("./build"):
                os.system("mkdir build")
            os.chdir("./build")
            os.system("cmake ..")

if __name__ == '__main__':
    main()