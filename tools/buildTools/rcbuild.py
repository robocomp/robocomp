#!/usr/bin/python

from __future__ import print_function
import os
import sys
import argparse
from workspace import workspace as WS

def build_component(bcomponent):
    paths = WS.find_component_src(bcomponent)
    if not paths:
        print("No such component exists")
        return False
    path = paths[0]
    srcpath = WS.find_workspace(path) + '/src'
    
    #ignore other components
    ignored_comps=[]
    for component in os.listdir(srcpath):
        if bcomponent == component or os.path.isdir(os.path.join(srcpath, component))==False or os.path.exists(os.path.join(srcpath,component,'.ignore_comp'))  :
            continue
        ignored_comps.append(component)
        os.system("touch " + os.path.join(srcpath,component,'.ignore_comp') )
    
    #build
    os.chdir(srcpath)
    os.chdir("../build")
    os.system("cmake ../src")
    
    #unignore other components
    for comp in ignored_comps:
        os.system("rm -f " + os.path.join(srcpath,comp,'.ignore_comp'))


def main():
    parser = argparse.ArgumentParser(description="configures and build components ")
    parser.add_argument('component', nargs='?', help='name of the component to build if omitted curent workspace is build')
    args = parser.parse_args()

    if not args.component:
        cpath = os.path.abspath('.')
        wspath = WS.find_workspace(cpath)
        if os.path.exists(".rc_workspace"):
            #if we are at the workspace base directory
            os.chdir("./build")
            os.system("cmake ../src")
        elif wspath:
            #if we are inside a workspace
            rest = cpath[len(wspath):].split('/')
            if len(rest) >= 3 and rest[1] =='src' :
                build_component(rest[2])
            else:
                os.chdir(wspath+"/build")
                os.system("cmake ../src")
        else:
            parser.error("This is not a valid robocomp workspace")
    else:
        component = args.component
        build_component(component)

if __name__ == '__main__':
    main()