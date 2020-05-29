#! /usr/bin/env python3
import argparse
import os
import sys

from prompt_toolkit import prompt
from prompt_toolkit.completion import PathCompleter
from prompt_toolkit.shortcuts import confirm
from prompt_toolkit.validation import Validator
from termcolor import colored

sys.path.append('/opt/robocomp/python')
from workspace import Workspace

def validate_in_range(value, valid_range):
    return value.isdigit() and int(value) in valid_range


class rccd:
    def __init__(self):
        self.ws = Workspace()

    def print_filtered_component(self, searched_component):
        components = self.ws.components
        if not components:
            print('rccd needs to have components dir configured.')
            answer = confirm('Do you want to configure it now?')
            if answer:
                new_path = prompt('Parent dir for components? ', completer=PathCompleter(), complete_while_typing=True,
                                  default=os.getenv('ROBOCOMP'))
                if new_path:
                    self.ws.update_robocomp_workspaces(new_path)
                    components = self.ws.components
            else:
                print("")
                return
        if components:
            options = list(filter(lambda x: searched_component.lower() in x.lower(), components))

            if len(options) == 1:
                print(f"{options[0]}\n\n")
            elif len(options) > 1:
                print(f"Options")
                print(f"[0] .")
                for index, option in enumerate(options):
                    print(f"[{index + 1}] {option}")

                validator = Validator.from_callable(
                    lambda x: validate_in_range(x, list(range(0, len(options) + 1))),
                    error_message=f'This input contains non valid number [0-{len(options)}]',
                    move_cursor_to_end=True)
                try:
                    selected = prompt('> ',
                                      validator=validator)
                    if selected == "0":
                        print(os.getcwd())
                    else:
                        print(options[int(selected) - 1])
                except KeyboardInterrupt:
                    print("")
            else:
                print("")
        else:
            print("\n")


if __name__ == '__main__':
    rccd = rccd()
    parser = argparse.ArgumentParser()
    parser.add_argument("name")
    args = parser.parse_args()
    try:
        rccd.print_filtered_component(args.name)
    except KeyboardInterrupt:
        print("")





