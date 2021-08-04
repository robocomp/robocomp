#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
from __future__ import print_function
import argparse, argcomplete
import os
import string
import sys

sys.path.append('/opt/robocomp/python')

from workspace import Workspace

''' edit files of robocomp compoent '''

ws = Workspace()

def complete_components(prefix, **kwargs):
    components = ws.components #WS.list_components_in_workspace(WS.workspace_paths)
    componentsname=[]
    for component in components:
        componentsname.append(component.split('/')[ len(component.split('/')) -1 ])
    return (componentname for componentname in componentsname if componentname.startswith(prefix))

def complete_files(prefix, parsed_args  , **kwargs):
    print("parsed_args.component[0]")
    filelist = ws.search_for_file(parsed_args.component[0] , ['*'])
    return ( filename[0] for filename in filelist if filename[0].startswith(prefix))
    # return 
    

def main():
    parser = argparse.ArgumentParser(description="directly edit a file within any component")
    parser.add_argument('component', nargs=1, help='name of the component').completer = complete_components
    parser.add_argument('file', nargs=1, help='name of the file of the given component you need to edit').completer = complete_files
    
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    sfile = args.file[0]
    component = args.component[0]

    filelist = ws.search_for_file(component,[str(sfile)])
    if len(filelist) == 0:
         parser.error("couldnt find the file {0} in {1} source ".format(sfile,component))
    
    editor = os.environ.get('EDITOR')
    if not editor:
        editor = 'vi'
    
    if len(filelist) == 1:
        command = editor +  " " + filelist[0][1]
    else:
        print("This is a non-unique file, please select one of the following:\n")
        count=1
        for fpath in filelist:
            print(str(count)+') '+fpath[1])
            count += 1
        option = input("please selact a file : ")
        command = editor + " " + filelist[option-1][1]

    try:
        #print(command)
        os.system(command)
    except Exception as openEx:
        print("Cant open the specifiled file for editing : %s" % (str(openEx)) )


if __name__ == '__main__':
    main()