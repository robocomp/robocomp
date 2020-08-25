#!/usr/bin/env python3
import argparse
import os
import sys
import tempfile

from prompt_toolkit import prompt
from prompt_toolkit.completion import PathCompleter
from prompt_toolkit.shortcuts import confirm
from prompt_toolkit.validation import Validator
from termcolor import colored

sys.path.append('/opt/robocomp/python')
from workspace import Workspace



def save_output(output):
    fd, path = tempfile.mkstemp("_rccd.output", text=True)
    file = os.fdopen(fd, 'w')
    file.write(output)
    file.close()


class rccd:
    def __init__(self):
        self.ws = Workspace()

    def save_filtered_component(self, searched_component):
        options = self.ws.find_components(searched_component)
        if options is not None:
            if len(options) == 1:
                save_output(f"{options[0]}")
            elif len(options) > 1:
                selected = self.ws.ask_for_path_selection(options)
                if selected is None:
                    save_output("")
                else:
                    save_output(selected)
            else:
                save_output("")
        else:
            save_output("")


if __name__ == '__main__':
    rccd = rccd()
    parser = argparse.ArgumentParser()
    parser.add_argument("name")
    args = parser.parse_args()
    try:
        rccd.save_filtered_component(args.name)
    except KeyboardInterrupt:
        save_output("")





