#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse

from pyaku import pyaku

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-a",
        "--append",
        action="store_true",
        dest="append",
        help="Append the given title to the directory title for the Tab title",
    )
    parser.add_argument(
        "-e",
        "--every",
        action="store_true",
        dest="every",
        help="Rename every open tab to the directory the session is in.",
    )
    parser.add_argument("title", nargs="?", help="Alternative title for the tab")
    parser.add_argument(
        "-s",
        "--save",
        action="store_true",
        dest="save",
        help="Save all the current tabs_by_name information to an script to restore session.",
    )
    args = parser.parse_args()
    yaku = pyaku.Yaku()
    if args.save:
        yaku.create_yakuake_start_shell_script()
    elif args.every:
        yaku.rename_all_tabs(args.title, args.append)
    else:
        yaku.rename_current_tab(args.title, args.append)

if __name__ == "__main__":
   main()
