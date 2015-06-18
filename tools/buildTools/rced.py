#!/usr/bin/python
from __future__ import print_function
import argparse
import os
import string
from workspace import workspace as WS



''' edit files of robocomp compoent '''

def main():
    parser = argparse.ArgumentParser(description="initialize a robocomp workspace")
    parser.add_argument('component', nargs=1, help='name of the component')
    parser.add_argument('file', nargs=1, help='name of the file of the given component you need to edit')
    args = parser.parse_args()

    sfile = args.file[0]
    component = args.component[0]

    filelist = WS.search_for_file(component,[str(sfile)])
    if len(filelist) == 0:
        parser.error("couldnt find the file {0} in {1} source ".format(sfile,component))
    
    editor = os.environ.get('EDITOR')
    if not editor:
        editor = 'vim'
    
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