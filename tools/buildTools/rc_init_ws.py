#!/usr/bin/python

from __future__ import print_function
import os
import sys
import argparse

sys.path.append('/opt/robocomp/python')


from workspace import workspace as WS

def main():
    parser = argparse.ArgumentParser(description="initialize a robocomp workspace")
    parser.add_argument('workspace', nargs='?', default='.', help='The path to an existing folder (default: .)')
    args = parser.parse_args()

    # verify that workspace folder exists
    workspace = os.path.abspath(args.workspace)
    if not os.path.isdir(workspace):
        parser.error('Workspace "%s" does not exist' % workspace)

    #verify that this is not an existing workspace
    if os.path.exists( os.path.join(workspace,".rc_workspace")):
        print('\nWorkspace "%s" is alreay an workspace' % workspace)
        if WS.register_workspace(workspace):
            print("Re-registred this workspace\n")
        return

    #try creating a workspace
    try:
        WS.init_ws(workspace)
    except Exception as e:
        sys.stderr.write(str(e))
        sys.exit(2)
    else:
        sys.stdout.write("sucessfully initialized robocomp workspace in %s \n" % (str(os.path.abspath(workspace))) )
        sys.stdout.write("To remove this workspace delete the file .rc_workspace\n")
if __name__ == '__main__':
    main()

