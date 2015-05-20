#!/usr/bin/python

from __future__ import print_function
import os
import sys
import argparse
import workspace as WS

def main():
    parser = argparse.ArgumentParser(description="initialize a robocomp workspace")
    parser.add_argument('workspace', nargs='?', default='.', help='The path to an existing folder (default: .)')
    args = parser.parse_args()

    # verify that workspace folder exists
    workspace = os.path.abspath(args.workspace)
    if not os.path.isdir(workspace):
        parser.error('Workspace "%s" does not exist' % workspace)

    #verify that this is not an existing workspace
    if os.path.exists( os.path.join(workspace,"toplevel.cmake")):
        parser.error('Workspace "%s" is alreay an workspace' % workspace)

    #try creating a workspace
    try:
        WS.rc_ws.init_ws(workspace)
    except Exception as e:
        sys.stderr.write(str(e))
        sys.exit(2)
    else:
        sys.stdout.write("sucessfully initialized robocomp workspace in %s \n" % (str(os.path.abspath(workspace))) )

if __name__ == '__main__':
    main()

