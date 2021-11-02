#!/usr/bin/env python3
import dataclasses
import json
import os
import re
from pathlib import Path

from prompt_toolkit.shortcuts import confirm
from prompt_toolkit.validation import Validator
from prompt_toolkit import prompt, PromptSession
from prompt_toolkit.completion import FuzzyCompleter, PathCompleter, WordCompleter
from rcworkspace.component_dir import ComponentDir
from termcolor import colored

''' Basic module which defines the workspace class
'''


def is_valid_dir(text: Path):
    return text.exists() and text.is_dir()


dir_validator = Validator.from_callable(
    is_valid_dir,
    error_message="Not a valid directory (doesn't exist or is not a dir).",
    move_cursor_to_end=True,
)


class Workspace:
    # TODO: convert to an unique dict with workspaces as key and components as values.
    workspace_paths = []
    components = {}
    interfaces = []

    ''' constructor'''

    def __init__(self):
        self.load_workspace()

    def __del__(self):
        pass

    def save_workspace(self):
        self.save_workspace_paths()
        self.save_components()

    def load_workspace(self):
        self.load_workspaces_paths()
        self.load_components()

    def find_components(self, searched_component, reg_exp=False):
        components_found = None
        if not self.components:
            print('No workspace have been configured')
        if self.components:
            if reg_exp:
                components_found = [component for c_path, component in self.components.items() if re.match(".*"+searched_component+".*", c_path)]
            else:
                components_found = [component for c_path, component in self.components.items() if str(searched_component) in c_path]
        return components_found

    def find_component(self, searched_component, interactive=False, reg_exp=False):
        component = self.components[searched_component] if searched_component in self.components else None
        if component is not None:
            return component
        components_found = self.find_components(searched_component, reg_exp=reg_exp)
        if components_found is not None:
            if len(components_found) == 1:
                component = components_found[0]
            elif len(components_found) > 1:
                if interactive:
                    selected = self.ask_for_path_selection(components_found)
                    if selected is not None:
                        component = selected
                else:
                    print(f"Found multiple components ({len(components_found)}) for {searched_component}")
                    print(f"Narrow the search with a more specific component name or path.")
        return component

    def find_component_path(self, searched_component, interactive=False):
        if component := self.find_component(searched_component, interactive):
            return component.path
        return None


    ''' search and return a dictionary of file paths given component name and file name
        if duplicate components are found, first to found is chosen
        component - name of component as string
        files     - list of files to search for; if files[0]=* all files are returned
        return    - list of tuple of filnames and path
    '''

    def search_for_file(self, component, sfiles):
        filedict = []
        allfiledict = []
        # aa = [('testcomp1.cdsl', '/home/nithin/tmp/rc/rc_ws/src/testcomp1/testcomp1.cdsl'), ('README.md', '/home/nithin/tmp/rc/rc_ws/src/testcomp1/README.md')]
        componentPath = self.find_component_src_path(component)

        for sfile in sfiles:
            for root, dirs, files in os.walk(componentPath):
                for tfile in files:
                    tmptuple = (tfile, os.path.join(root, tfile))
                    allfiledict.append(tmptuple)
                if sfile in files:
                    tmptuple = (sfile, os.path.join(root, sfile))
                    filedict.append(tmptuple)

        if sfile[0] == '*':
            return allfiledict
        else:
            return filedict

    '''check if the given path is inside a workspace'''

    def find_in_workspaces(self, path):
        for ws in self.workspace_paths:
            if path[:len(ws)] == ws:
                return ws
        return False

    def list_components_names(self):
        components_names = []
        for component in self.components:
            components_names.append(component.name)
        return components_names

    def list_filtered_components_names(self, prefix):
        names = self.list_components_names()
        return (name for name in names if name.startswith(prefix))

    def get_recursive_components_in_dir(self, initial_path: Path, print_path=False):
        print("get_recursive_components_in_dir", list(map(str, self.workspace_paths)))
        components_parents_dirs = {}
        for subdir in initial_path.resolve().rglob("*"):
            if subdir.is_dir() and (comp := ComponentDir.create_component(subdir)):
                components_parents_dirs[subdir] = comp
                if print_path:
                    print(f'Found {str(subdir)}')
        return components_parents_dirs

    def get_recursive_interfaces_in_dir(self, initial_path, print_path=False):
        interface_files = []
        if (initial_path := Path(initial_path)).is_dir():
            interface_files += list(map(str, initial_path.rglob("*.idsl")))
        return interface_files

    def ask_for_path_selection(self, components_dir_list):
        print("Options")
        print("[0] .")
        for index, option in enumerate(components_dir_list):
            print(f"[{index + 1}] {option.path}")

        validator = Validator.from_callable(
            lambda x: validate_in_range(x, list(range(0, len(components_dir_list) + 1))),
            error_message=f'This input contains non valid number [0-{len(components_dir_list)}]',
            move_cursor_to_end=True)
        try:
            selected = prompt('> ',
                              validator=validator)
            if selected == "0":
                return os.getcwd()
            else:
                return components_dir_list[int(selected) - 1]
        except KeyboardInterrupt:
            return None

    def ask_for_path(self, initial):
        session = PromptSession(u"> ", completer=FuzzyCompleter(PathCompleter()))
        response = Path(session.prompt("Path to find new components\n> ",
                                       complete_while_typing=True,
                                       default=initial,
                                       pre_run=session.default_buffer.start_completion,
                                       validator=dir_validator))
        return response

    def search_parent_in_workspaces(self, path: Path):
        path = path.absolute()
        for workspace_path in self.workspace_paths:
            common_path = Path(os.path.commonpath([path, workspace_path]))
            if common_path == workspace_path:
                return workspace_path
        return None

    def add_workspace(self, initial: Path = None, accept_all=False, interactive=False) -> None:
        if initial is None:
            initial = Path.cwd()
        else:
            initial = Path(initial).absolute()
        existing_workspace = self.search_parent_in_workspaces(initial)
        if existing_workspace is None:
            if accept_all or not interactive:
                response = initial
            else:
                response = self.ask_for_path(initial)
            if response in self.workspace_paths:
                print(f"{response} already exist in workspaces")
                return

            new_components = self.get_recursive_components_in_dir(response)
            new_interfaces = self.get_recursive_interfaces_in_dir(response)

            if len(new_components) == 0:
                print(f"No component found in {response}. Workspaces not updated.")
                return
            else:
                print("%s\n%d components found in %s" % (colored('\n'.join(new_components), 'green'),
                                                         len(new_components),
                                                         response))
            if len(new_interfaces) == 0:
                print(f"No interface file found in {response}. Workspaces not updated.")
                return
            else:
                print("%s\n%d interface files found in %s" % (colored('\n'.join(new_interfaces), 'green'),
                                                         len(new_interfaces),
                                                         response))

            if accept_all or interactive is False:
                answer = True
            else:
                answer = confirm(f'Do you want to add {response} to workspaces?')

            if answer:
                print(f"{response} added to workspaces")
                self.components.update(new_components)
                self.interfaces.append(new_interfaces)
                self.workspace_paths.append(response)
                self.save_workspace()
            else:
                print("Workspaces not updated.")
        else:
            print(f"{initial} is already part of an existing workspace ({existing_workspace})")
            self.update_components_in_workspaces(existing_workspace)
            self.update_interfaces_in_workspaces(existing_workspace)

    def update_components_in_workspaces(self, workspace=None):
        old_components_paths = list(map(Path, self.components.keys()))
        if workspace is None:
            print("update_components_in_workspaces", list(map(str, self.workspace_paths)))
            for existing_workspace in self.workspace_paths:
                res = self.get_recursive_components_in_dir(existing_workspace)
                self.components.update(res)
            print(f"Updating workspaces")
        else:
            self.components.update(self.get_recursive_components_in_dir(workspace))
            print(f"Updating workspace {workspace}")
        new_components = list(self.components.keys() - old_components_paths)
        print(f"Found {colored(len(new_components), 'green')} new components")
        # if len(new_components) > 0:
        #     print(colored('\n'.join(str(new_components)), 'green'))
        self.save_workspace()

    def update_interfaces_in_workspaces(self, workspace=None):
        old_interfaces = self.interfaces
        if workspace is None:
            for existing_workspace in self.workspace_paths:
                res = self.get_recursive_interfaces_in_dir(existing_workspace)
                self.interfaces = list(set(res+old_interfaces))
            print(f"Updating workspaces")
        else:
            res = self.get_recursive_interfaces_in_dir(workspace)
            self.interfaces = list(set(res + old_interfaces))
            print(f"Updating workspace {workspace}")
        new_interfaces = list(set(self.interfaces) - set(old_interfaces))
        print(f"Found {colored(len(new_interfaces), 'green')} new interfaces")
        if len(new_interfaces) > 0:
            print(colored('\n'.join(new_interfaces), 'green'))
        self.save_workspace()

    def delete_workspace(self, workspace_path: Path = None, interactive=False):
        if workspace_path is None:
            workspace_path = Path.cwd()
        else:
            workspace_path = Path(workspace_path)

        if interactive:
            session = PromptSession(u"> ", completer=FuzzyCompleter(WordCompleter(self.workspace_paths)))
            response = session.prompt("Path to delete\n> ",
                                      complete_while_typing=True,
                                      default=workspace_path,
                                      pre_run=session.default_buffer.start_completion)
        else:
            if workspace_path.exists():
                response = workspace_path
        if response in self.workspace_paths:
            self._delete_components_in_workspace(response)
            self.workspace_paths.remove(response)
            self.save_workspace()
            print(f"{response} deleted from workspaces.")
        else:
            print(f"{response} not found in workspaces.")

    def _delete_components_in_workspace(self, workspace: Path):
        self.components = {k: v for k, v in self.components.items() if k.startswith(str(workspace))}

    def clear_all(self, interactive=False):
        if interactive:
            print(
                f"You are about to remove {colored(len(self.components), 'red')} from {colored(len(self.workspace_paths), 'red')} workspaces.")
            print(f"This action only remove the references to this workspaces and components for rccommands.")
            print(colored("NO actual folders nor files are going to be removed.", 'red'))
            answer = confirm(f"Are you sure that you want to do this?")
        else:
            answer = True
        if answer:
            self.components = {}
            self.workspace_paths = []
            print("All workspaces have been removed.")
        else:
            print("No workspaces have been deleted.")
        self.save_workspace()

    def list_workspaces(self):
        for workspace in self.workspace_paths:
            print(f"Workspace {workspace}")
            for component in self.components:
                if component.startswith(str(workspace)):
                    print(f"\tComponent {component}")

    def interactive_workspace_init(self, initial_path):
        def common_prefix(path_list):
            result = {}
            to_ignore = ""

            for path in sorted(path_list, reverse=True):
                path_parts = path.split('/')
                previous_counter = 0
                previous_path = ""
                for part_index in range(len(path_parts)):
                    counter = 0
                    last_path = '/'.join(path_parts[:len(path_parts) - part_index])
                    last_path = last_path[len(to_ignore):]
                    if last_path and to_ignore + last_path not in result:
                        for second_path in path_list:
                            if last_path in second_path:
                                counter += 1
                        if counter == previous_counter:
                            del result[to_ignore + previous_path]
                        previous_counter = counter
                        previous_path = last_path
                        if counter == len(path_list):
                            to_ignore = to_ignore + last_path
                        elif last_path and counter < len(path_list):
                            result[to_ignore + last_path] = counter

            return result

        components_parents_dirs = self.get_recursive_components_in_dir(initial_path)
        best_guess = common_prefix([str(dir_l) for dir_l in components_parents_dirs.keys()])
        new_workspaces = []
        print(f"Found {len(best_guess)} possibles components in {colored(initial_path, 'green')}")
        print(f"======")
        ignored = []
        for path in sorted(best_guess, key=lambda x: calculate_path_value(x, best_guess[x]), reverse=True):
            if best_guess[path] <= 1:
                continue
            end = False
            next = False
            for parent in new_workspaces:
                if path.startswith(parent):
                    print(
                        f"{colored(path, 'green')} ignored because you already selected parent {colored(parent, 'green')}")
                    next = True
                    break
            for parent in ignored:
                if path.startswith(parent):
                    print(
                        f"{colored(path, 'red')} ignored because you decided to ignore subdirs of {colored(parent, 'red')}")
                    next = True
                    break
            if next:
                continue
            while not end:
                print(f"Found {best_guess[path]} components in {colored(path, 'green')}")
                response = input(f"Do you wanna add to workspace? {colored('Yes/No/Ignore/End', 'cyan')} ").strip()
                if "yes" == response.lower() or "y" == response.lower():
                    new_workspaces.append(path)
                    break
                if "ignore" == response.lower() or "i" == response.lower():
                    to_ignore = prompt('> ',
                                       completer=FuzzyCompleter(PathCompleter()),
                                       complete_while_typing=True,
                                       default=path)
                    to_ignore = to_ignore.rstrip('/')
                    if not to_ignore:
                        to_ignore = path
                    ignored.append(to_ignore)
                    break
                elif "no" == response.lower() or "n" == response.lower():
                    break
                elif "end" == response.lower() or "e" == response.lower():
                    print("Finishing initialization of workspaces")
                    end = True
                else:
                    print('Invalid response (y/n/i/e)')
            if end:
                break
        self.workspace_paths = list(map(Path, new_workspaces))
        self.update_components_in_workspaces()


    def _save_attr(self, attr, filename):
        home = os.path.expanduser("~")
        config_file_path = os.path.join(home, f".config/RoboComp/rc_{filename}.json")
        if not os.path.exists(os.path.join(home, ".config/RoboComp")):
            config_path = os.path.join(home, ".config/RoboComp")
            os.makedirs(config_path)
            os.chmod(config_path, 0o777)
        try:
            config_file = open(config_file_path, "w")
            json.dump(attr, config_file, cls=EnhancedJSONEncoder)
            config_file.close()
        except FileNotFoundError:
            pass

    def _load_attr(self, filename):
        home = os.path.expanduser("~")
        config_file_path = os.path.join(home, f".config/RoboComp/rc_{filename}.json")

        if not os.path.exists(os.path.join(home, ".config/RoboComp")):
            config_path = os.path.join(home, ".config/RoboComp")
            try:
                os.makedirs(config_path)
                os.chmod(config_path, 0o777)
            except PermissionError as e:
                print(f"Could not create {config_path} directory.")
                print(f"{e}")
                return []
        try:
            config_file = open(config_file_path, "r")
            attr = json.load(config_file)
            attr = [x.strip() for x in attr]
            config_file.close()
            return attr
        except FileNotFoundError:
            return []
        except json.JSONDecodeError as e:
            print(f"Invalid json format in {config_file_path}")
            print(f"{e}")
            return []

    def load_workspaces_paths(self):
        self.workspace_paths = list(map(Path, self._load_attr("workspace_paths")))

    def load_components(self):
        self.components = {c_path: ComponentDir.create_component(Path(c_path)) for c_path in self._load_attr('components')}

    def save_workspace_paths(self):
        print(list(map(str, self.workspace_paths)))
        self._save_attr(list(map(str, self.workspace_paths)), 'workspace_paths')

    def save_components(self):
        self._save_attr(list(self.components.keys()), 'components')


def folder_is_repository(path):
    git_path = os.path.join(path, '.git')
    if os.path.isdir(git_path):
        head_file = os.path.join(git_path, 'HEAD')
        if os.path.isfile(head_file):
            return True
    return False


def calculate_path_value(path, initial_value):
    new_value = len(path.split(os.sep)) * initial_value
    if folder_is_repository(path):
        new_value *= 2
    return new_value


def validate_in_range(value, valid_range):
    return value.isdigit() and int(value) in valid_range


class EnhancedJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, Path):
            return str(o)
        if dataclasses.is_dataclass(o):
            return dataclasses.asdict(o)
        return super().default(o)
