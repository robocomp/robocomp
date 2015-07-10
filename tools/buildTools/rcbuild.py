#!/usr/bin/python

from __future__ import print_function
import os
import sys
import argparse
from workspace import workspace as WS

def build_component(bcomponent,install):
    paths = WS.find_component_src(bcomponent)
    if not paths:
        print("No such component exists")
        return False
    path = paths[0]
    srcpath = WS.find_workspace(path) + '/src'
    
    #ignore other components
    ignored_comps=[]
    for component in os.listdir(srcpath):
        if bcomponent == component or os.path.isdir(os.path.join(srcpath, component))==False or os.path.exists(os.path.join(srcpath,component,'IGNORE_COMP'))  :
            continue
        ignored_comps.append(component)
        os.system("touch " + os.path.join(srcpath,component,'IGNORE_COMP') )
    
    #build
    os.chdir(srcpath)
    os.chdir("../build")
    os.system("cmake ../src")
    if install == True:
        os.system("sudo make install")
    else:
        os.system("make")
    
    #unignore other components
    for comp in ignored_comps:
        os.system("rm -f " + os.path.join(srcpath,comp,'IGNORE_COMP'))

def build_docs(component,install=False,installpath='/opt/robocomp'):
    paths = WS.find_component_src(component)
    if not paths:
        print("No such component exists")
        return False
    path = paths[0]
    os.chdir(path)
    if install == True:
        try:
            os.system('mkdir -p '+installpath+'/doc')
            os.system(' sudo cp -R doc/html '+installpath+'/doc/'+string.lower(component))
        except Exception, e:
            raise RuntimeError("couldnt install doc files {0}".format(e))
    else:
        try:
            os.system('doxygen Doxyfile')    
        except Exception as e:
            raise RuntimeError("couldnt generate doc files {0}".format(e))

def main():
    parser = argparse.ArgumentParser(description="configures and build components ")
    group = parser.add_mutually_exclusive_group()
    parser.add_argument('component', nargs='?', help='name of the component to build, if omitted curent workspace is build')
    group.add_argument('-i','--install', action = 'store_true' , help="build and install the components")
    group.add_argument('--doc', action = 'store_true' , help="generate documentation")
    group.add_argument('--installdoc', action = 'store_true' , help="install documentation")
    
    args = parser.parse_args()

    if not args.component:
        cpath = os.path.abspath('.')
        wspath = WS.find_workspace(cpath) #see if path consisits of an workspace 
        
        if wspath:
            #if we are inside a workspace
            
            rest = cpath[len(wspath):].split('/')
            if len(rest) >= 3 and rest[1] =='src' :
                #if we are inside a component source directory
                if args.doc or args.installdoc:
                    build_docs(rest[2],args.installdoc)
                else:
                    build_component(rest[2],args.install)
            else:
                os.chdir(wspath+"/build")
                os.system("cmake ../src")
                if args.installdoc or args.doc:
                    print("\nDocs can oly be generated for one component at a time")
                else:
                    if args.install:
                        os.system('sudo make install')
                    else:
                        os.system('make')
        else:
            parser.error("This is not a valid robocomp workspace")
    
    else:
        component = args.component
        if args.doc or args.installdoc:
            build_docs(component,args.installdoc)
        else:
            build_component(component,args.install)

if __name__ == '__main__':
    main()