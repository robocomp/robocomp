#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import sys
import argparse
sys.path.append("/opt/robocomp/python")
import yaku_lib

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--append', action='store_true', dest="append",
                        help="append the given name to the directory name for the Tab title")
    parser.add_argument('name', nargs='?', help="Alternative name for the tab")
    parser.add_argument('-s', '--save', action='store_true', dest="save",
                        help="save all the current tabs information to an script to restore session.")
    args = parser.parse_args()
    yaku = yaku_lib.Yaku()
    if args.save:
        yaku.create_yakuake_start_shell_script()
    else:
        yaku.rename_current_tab(args.name, args.append)