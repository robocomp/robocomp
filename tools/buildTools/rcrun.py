#!/usr/bin/python

from __future__ import print_function
import argparse
import os
import string
import workspace

def main():
    parser = argparse.ArgumentParser(description="Tool for locating and running a robocomp component")
    group = parser.add_mutually_exclusive_group(required = True)
    group.add_argument('component', nargs='?', help='start a component')
    group.add_argument('-s', '--start', nargs = 1, help="start a component")
    group.add_argument('-st', '--stop', nargs = 1, help="stop a component")
    group.add_argument('-fst', '--fstop', nargs = 1, help=" force start a component")
    parser.add_argument('-d', '--debug', action = 'store_true' , help="start a component")
    parser.add_argument('-c', '--config', nargs = 1 , help="use custom ice config file (relative path)")
    args = parser.parse_args()

    #if stop command no need for searching and all
    if args.stop:
        command = "killall " + str(args.stop[0])
        os.system(command)
        return
    elif args.fstop:
        command = "killall -9 " + str(args.fstop[0])
        os.system(command)
        return
    print(args.start)
    if args.start:
        component = str(args.start[0])
    elif args.component:
        component = args.component
    else:
        parser.error("No component specified")

    print( "component " + str(component) + "\n")
    
    ##search for the component
    componentPath = workspace.rc_ws.find_component_exec(component)
    if not componentPath:
        print("couldnt find the component %s in any of the workspaces" % (component))
        return
    
    print(str(componentPath))

    #execute the appropriate command
    ice_config = '../etc/generic_config'
    if args.config:
        ice_config = args.config

    if args.start:
        command = componentPath + "/" + component + " --Ice.Config=" + ice_config
    else:
        command = componentPath + "/" + component + " --Ice.Config=" + ice_config

    os.system(command)

if __name__ == '__main__':
    main()

