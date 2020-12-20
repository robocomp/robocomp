#!/usr/bin/env python3
import json
import os
import string
from collections import defaultdict

from prompt_toolkit.shortcuts import confirm
from prompt_toolkit.validation import Validator
from termcolor import colored

''' Basic module which defines the workspace class
'''

from prompt_toolkit import prompt, PromptSession
from prompt_toolkit.completion import FuzzyCompleter, PathCompleter, WordCompleter


def is_valid_dir(text):
    return os.path.exists(text) and os.path.isdir(text)


dir_validator = Validator.from_callable(
    is_valid_dir,
    error_message="Not a valid directory (doesn't exist or is not a dir).",
    move_cursor_to_end=True,
)

class Workspace:
    #TODO: convert to an unique dict with workspaces as key and components as values.
    workspace_paths = []
    components = []
    numOfws = 0
    robocomp_dir = ''

    ''' constructor'''
    def __init__(self):
        self.workspace_paths = self._load_attr('workspace_paths')
        self.components = self._load_attr('components')

    def __del__(self):
        pass

    def save_workspace(self):
        self._save_attr(self.workspace_paths, 'workspace_paths')
        self._save_attr(self.components, 'components')

    def find_components(self, searched_component):
        if not self.components:
            #TODO: add options to interactive search (interactive_workspace_init) or add (add_workspace)
            print('rccd needs to have components dir configured.')
            answer = confirm('Do you want to configure it now?')
            if answer:
                new_path = prompt('Parent dir for components? ', completer=PathCompleter(), complete_while_typing=True,
                                  default=os.getenv('ROBOCOMP'))
                if new_path:
                    self.interactive_workspace_init(new_path)
        if self.components:
            options = list(filter(lambda x: searched_component.lower() in x.lower(), self.components))
            return options
        return None

    def find_component(self, component):
        component_dir = ""
        options = self.find_components(component)
        if options is not None:
            if len(options) == 1:
                component_dir = options[0]
            elif len(options) > 1:
                selected = self.ask_for_path_selection(options)
                if selected is not None:
                    component_dir = selected
        if component_dir and os.path.exists(component_dir) and os.path.isdir(component_dir):
            return component_dir

    ''' find the directory containing component executable'''
    def find_component_exec_file(self, component):
        if os.path.exists(component):
            component_dir = component
            component = component.split(os.path.sep)[-1]
        else:
            component_dir = self.find_component(component)

        if component_dir:
            src_path = os.path.join(component_dir.strip(),'src')
            bin_path = os.path.join(component_dir.strip(),'bin')
            if os.path.isdir(bin_path):
                bin_file_path = os.path.join(bin_path, component)
                if os.access(bin_file_path, os.X_OK):
                    return bin_file_path
            elif os.path.isdir(src_path):
                python_main_path = os.path.join(src_path, component+".py")
                if os.path.exists(python_main_path):
                    return python_main_path
        return ""

    def find_component_bin_path(self, component):
        if os.path.exists(component):
            bin_path = component
        else:
            bin_path = self.find_component(component)
        if bin_path:
            bin_path = os.path.join(bin_path, 'bin')
            if os.path.exists(bin_path) and os.path.isdir(bin_path):
                return bin_path
        return None

    ''' find component source directory
        component - component name
        return    - the component directory in src
    '''
    def find_component_src_path(self, component):
        if os.path.exists(component):
            src_path = component
        else:
            src_path = self.find_component(component)
        if src_path:
            src_path = os.path.join(src_path, 'src')
            if os.path.exists(src_path) and os.path.isdir(src_path):
                return src_path
        return None

    def find_component_etc_path(self, component):
        if os.path.exists(component):
            etc_path = component
        else:
            etc_path = self.find_component(component)
        if etc_path:
            etc_path = os.path.join(etc_path, 'etc')
            if os.path.exists(etc_path) and os.path.isdir(etc_path):
                return etc_path
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
        #aa = [('testcomp1.cdsl', '/home/nithin/tmp/rc/rc_ws/src/testcomp1/testcomp1.cdsl'), ('README.md', '/home/nithin/tmp/rc/rc_ws/src/testcomp1/README.md')]
        componentPath = self.find_component_src_path(component)

        for sfile in sfiles:
            for root, dirs, files in os.walk(componentPath):                
                for tfile in files:
                    tmptuple = (tfile,os.path.join(root, tfile))
                    allfiledict.append(tmptuple)
                if sfile in files:
                    tmptuple = (sfile,os.path.join(root, sfile))
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
            components_names.append(component.lstrip('/').split('/')[-1])
        return components_names

    def list_filtered_components_names(self, prefix):
        names = self.list_components_names()
        return (name for name in names if name.startswith(prefix))

    ''' return all components paths given workspaces'''
    def list_components_in_workspace(self, ws_paths):
        components = []
        for ws in ws_paths:
            if self.find_in_workspaces(ws)==False:
                continue
            srcpath = ws + '/src'
            for component in os.listdir(srcpath):
                if os.path.isdir(os.path.join(srcpath, component))==False:
                    continue
                components.append(os.path.join(srcpath, component))
        return components

    #TODO: Create method to check if a path is a component
    def path_is_component(self, path):
        if self.find_in_workspaces(path):
            return any([component.lstrip('/').startswith(path.lstrip('/')) for component in self.components])
        return False

    def get_recursive_components_in_dir(self, initial_path, print_path=False):
        components_parents_dirs = defaultdict(list)
        for subdir, dirs, files in os.walk(initial_path):
            if 'CMakeLists.txt' in files and 'etc' in dirs \
                    and not subdir.startswith('test_')\
                    and '.local/share/Trash' not in subdir:
                for file in files:
                    if file.endswith(".cdsl"):
                        filepath= os.path.join(subdir,file)
                        components_parents_dirs[subdir].append(filepath)
                        if print_path:
                            print(f'Found {filepath}')
        return components_parents_dirs

    def ask_for_path_selection(self, components_dir_list):
        print("Options")
        print("[0] .")
        for index, option in enumerate(components_dir_list):
            print(f"[{index + 1}] {option}")

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

    def search_parent_in_workspaces(self, path):
        path = os.path.abspath(path)
        for workspace_path in self.workspace_paths:
            common_path = os.path.commonpath([path,workspace_path])
            if len(common_path) == len(workspace_path):
                return workspace_path
        return ""

    def add_workspace(self, initial=None, accept_all=False):
        if initial is None:
            initial = os.getcwd()
        else:
            initial = os.path.abspath(initial)
        existing_workspace = self.search_parent_in_workspaces(initial)
        if not existing_workspace:
            if accept_all:
                response = initial
            else:
                session = PromptSession(u"> ", completer=FuzzyCompleter(PathCompleter()))
                response = session.prompt("Path to find new components\n> ",
                                          complete_while_typing=True,
                                          default=initial,
                                          pre_run=session.default_buffer.start_completion,
                                          validator=dir_validator)
            # Check the final '/' characters
            if response in self.workspace_paths:
                print(f"{response} already exist in workspaces")
                return

            new_components = self.get_recursive_components_in_dir(response)

            print("%s\n%d components found in %s" % ( colored('\n'.join(new_components), 'green'),
                                                     len(new_components),
                                                     response))
            if len(new_components) == 0:
                print(f"No component found in {response}. Workspaces not updated.")
                return

            if accept_all:
                answer = True
            else:
                answer = confirm(f'Do you want to add {response} to workspaces?')

            if answer:
                print(f"{response} added to workspaces")
                self.components += new_components
                self.workspace_paths.append(response)
                self.save_workspace()
            else:
                print("Workspaces not updated.")
        else:
            print(f"{initial} is already part of an existing workspace ({existing_workspace})")
            self.update_components_in_workspaces(existing_workspace)

    def update_components_in_workspaces(self, workspace=None):
        old_components = set(self.components)
        self.components = []
        if workspace is None:
            for workspace in self.workspace_paths:
                self.components += self.get_recursive_components_in_dir(workspace)
            print(f"Updating workspaces")
        else:
            self.components += self.get_recursive_components_in_dir(workspace)
            print(f"Updating workspaces {workspace}")
        new_components = list(set(self.components) - old_components)
        print(f"Found {colored(len(new_components), 'green')} new components")
        print(colored('\n'.join(new_components), 'green'))
        self.save_workspace()


    def delete_workspace(self, keyword):
        session = PromptSession(u"> ", completer=FuzzyCompleter(WordCompleter(self.workspace_paths)))
        if not keyword:
            keyword = os.getcwd()
        else:
            keyword = os.path.abspath(keyword)
        response = session.prompt("Path to delete\n> ",
                                  complete_while_typing=True,
                                  default=keyword,
                                  pre_run=session.default_buffer.start_completion)
        if response in self.workspace_paths:
            self._delete_components_in_workspace(response)
            self.workspace_paths.remove(response)
            self.save_workspace()

    def _delete_components_in_workspace(self, workspace):
        self.components = list(filter(lambda x: not x.startswith(workspace), self.components))

    def clear_all(self):
        print(f"You are about to remove {colored(len(self.components),'red')} from {colored(len(self.workspace_paths), 'red')} workspaces.")
        print(f"This action only remove the references to this workspaces and components for rccommands.")
        print(colored("NO actual folders nor files are going to be removed.", 'red'))
        answer = confirm(f"Are you sure that you want to do this?")
        if answer:
            self.components = []
            self.workspace_paths = []
            print("All workspaces have been removed.")
        else:
            print("No workspaces have been deleted.")
        self.save_workspace()

    def list_workspaces(self):
        for workspace in self.workspace_paths:
            print(f"Workspace {workspace}")
            for component in self.components:
                if component.startswith(workspace):
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
                    last_path = '/'.join(path_parts[:len(path_parts)-part_index])
                    last_path = last_path[len(to_ignore):]
                    if last_path and to_ignore+last_path not in result:
                        for second_path in path_list:
                            if last_path in second_path:
                                counter += 1
                        if counter == previous_counter:
                            del result[to_ignore + previous_path]
                        previous_counter = counter
                        previous_path = last_path
                        if counter == len(path_list):
                            to_ignore = to_ignore+last_path
                        elif last_path and counter < len(path_list):
                            result[to_ignore+last_path] = counter


            return result


        components_parents_dirs = self.get_recursive_components_in_dir(initial_path)

        #
        best_guess = common_prefix(list(components_parents_dirs.keys()))
        new_workspaces = []
        print(f"Found {len(best_guess)} possibles components in {colored(initial_path, 'green')}")
        print(f"======")
        ignored = []
        for path in sorted(best_guess, key=lambda x: calculate_path_value(x, best_guess[x]), reverse=True):
            print(len(path.split('/'))*best_guess[path])
            end = False
            next = False
            for parent in new_workspaces:
                if path.startswith(parent):
                    print(f"{colored(path, 'green')} ignored because you already selected parent {colored(parent, 'green')}")
                    next = True
                    break
            for parent in ignored:
                if path.startswith(parent):
                    print(f"{colored(path, 'red')} ignored because you decided to ignore subdirs of {colored(parent, 'red')}")
                    next = True
                    break
            if next:
                continue
            while not end:
                print(f"Found {best_guess[path]} components in {colored(path, 'green')}")
                response = input(f"Do you wanna add to workspace? {colored('Yes/No/Ignore/End', 'cyan')} ")
                if "yes" in response.lower() or "y" in response.lower():
                    new_workspaces.append(path)
                    break
                if "ignore" in response.lower() or "i" in response.lower():
                    to_ignore = prompt('> ',
                                  completer=FuzzyCompleter(PathCompleter()),
                                  complete_while_typing=True,
                                  default=path)
                    to_ignore = to_ignore.rstrip('/')
                    if not to_ignore:
                        to_ignore = path
                    ignored.append(to_ignore)
                    break
                elif "no" in response.lower() or "n" in response.lower():
                    break
                elif "end" in response.lower() or "e" in response.lower():
                    print("Finishing initialization of workspaces")
                    end = True
                else:
                    print('Invalid response (y/n/i/e)')
            if end:
                break
        self.workspace_paths = new_workspaces
        self.components = list(filter(lambda x: any(x.startswith(workspace)for workspace in self.workspace_paths), components_parents_dirs))

        self.save_workspace()

    def _save_attr(self, attr, filename):
        home = os.path.expanduser("~")
        config_file_path = os.path.join(home, f".config/RoboComp/rc_{filename}.json")
        if not os.path.exists(os.path.join(home, ".config/RoboComp")):
            os.makedirs(os.path.join(home, ".config/RoboComp"))

        try:
            config_file = open(config_file_path, "w")
            json.dump(attr, config_file)
            config_file.close()
        except FileNotFoundError:
            pass


    def _load_attr(self, filename):
        home = os.path.expanduser("~")
        config_file_path = os.path.join(home, f".config/RoboComp/rc_{filename}.json")

        if not os.path.exists(os.path.join(home, ".config/RoboComp")):
            os.makedirs(os.path.join(home, ".config/RoboComp"))

        try:
            config_file = open(config_file_path, "r")
            attr = json.load(config_file)
            attr = [x.strip() for x in attr]
            config_file.close()
            return attr
        except FileNotFoundError:
            return []

def folder_is_repository(path):
    git_path = os.path.join(path, '.git')
    if os.path.isdir(git_path):
        head_file = os.path.join(git_path, 'HEAD')
        if os.path.isfile(head_file):
            return True
    return False

def calculate_path_value(path, initial_value):
    new_value = len(path.split(os.sep))*initial_value
    if folder_is_repository(path):
        new_value *= 2
    return new_value

def validate_in_range(value, valid_range):
    return value.isdigit() and int(value) in valid_range
