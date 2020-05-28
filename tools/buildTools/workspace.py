#!/usr/bin/python
import json
import os
import shutil
import string
from collections import defaultdict
from copy import deepcopy
from pprint import pprint

from termcolor import colored

''' Basic module which defines the workspace class
'''

from prompt_toolkit import prompt
from prompt_toolkit.completion import Completer, Completion, FuzzyCompleter


class MyCustomCompleter(Completer):
    def __init__(self, path):
        super(MyCustomCompleter, self).__init__()
        self.path = path
    def get_completions(self, document, complete_event):
        reference = document.current_line
        if self.path.startswith(reference):
            next_bar = self.path.find('/', len(reference)+1)
            if next_bar != -1:
                suggestion = self.path[len(reference):next_bar]
            else:
                suggestion = self.path[len(reference):]
            yield Completion(suggestion, start_position=0)


class Workspace:
    
    workspace_paths = []
    components = []
    numOfws = 0
    robocomp_dir = ''

    ''' constructor'''
    def __init__(self):
        self.workspace_paths = self.load_attr('workspace_paths')
        self.components = self.load_attr('components')
    
    def update_pathlist(self):
        home = os.path.expanduser("~")
        config_file_path = os.path.join(home,".config/RoboComp/rc_workspace.config")
        
        if not os.path.exists(os.path.join(home,".config/RoboComp")):
            os.makedirs(os.path.join(home,".config/RoboComp"))
        
        #add all folders in components directory as 
        #if not os.path.isfile(config_file_path):
        #    config_file = open(config_file_path, 'a').close()
        #    for folders in os.listdir(os.path.join(home,'robocomp/components')):
        #        if os.path.isdir(os.path.join(path,str(folders))):
        #            config_file.write( os.path.join(path,str(folders)) )

        try:
            config_file = open(config_file_path,"r")    
            self.workspace_paths = config_file.readlines()
            self.workspace_paths = [ string.strip(x) for x in self.workspace_paths ]
            config_file.close()            
        except FileNotFoundError:
            pass
    
    def register_workspace(self,workspacePath):
        if string.strip(workspacePath) not in self.workspace_paths:
            #update the list of workspaces
            home = os.path.expanduser("~")
            if not os.path.exists(os.path.join(home,".config/RoboComp")):
                os.makedirs(os.path.join(home,".config/RoboComp"))
            config_file = open(os.path.join(home,".config/RoboComp/rc_workspace.config"),"a+")
            config_file.write(workspacePath)
            config_file.write("\n")
            config_file.close()
            return True
        else:
            return False

    '''initiate a catkin workspace at the given path'''
    def init_ws(self, ws_path):
        
        def create_toplevel(src_path):
            src = os.getenv('ROBOCOMP','/opt/robocomp') + '/cmake/toplevel.cmake'
            dst = os.path.join(src_path,"CMakeLists.txt")
            
            #test if path exists
            if not os.path.exists(src):
                raise RuntimeError("Couldn't find toplevel cmake, make sure ROBOCOMP is properly installed\n")
            
            #try to create simlink
            try:
                os.symlink(src,dst)
            except Exception as symlinkEx:
                try:
                    shutil.copyfiles(src,dst)
                except Exception as copyEx:
                    raise RuntimeError('Could neither copy or simlink %s to %s : \n %s \n %s' % (src,dst,str(symlinkEx),str(copyEx)))

        #if already in a workspace, throw error
        if self.find_workspace(ws_path):
            raise RuntimeError("Sorry, you can't create workspace inside an existing one \n")

        try:
            os.system('touch {0}/{1}'.format(ws_path,str(".rc_workspace")))
            dirs = ["src","build","install"]
            for dir in dirs:
                dir_path = os.path.join(ws_path,dir)
                if not os.path.exists(dir_path):
                    os.makedirs(dir_path)
            os.system('touch {0}/install/{1}'.format(ws_path,str(".rc_install")))
        except Exception as createEx:
            raise RuntimeError("Couldn't create files/folders in current directory: \n %s " % (str(createEx)))

        pathstr = str(os.path.abspath(ws_path))
        
        #copy all files in the current dir into src dir
        print("Copying all files in workspace directory into src/ directory\n")
        for file in os.listdir(pathstr):
            if file in dirs or file in [".rc_workspace"] : continue
            try:
                os.system("mv ./" + file + " ./src")
            except Exception as copyEx:
                print("Couldn't copy all files\n")

        create_toplevel(ws_path+'/src')
        self.register_workspace(pathstr)

    ''' find the directory containing component executable'''
    def find_component_exec(self, component):
        componentPath = ''
        for path in self.workspace_paths:
            #path = string.strip(path) + '/devel'  for now we are not shifting the executables
            path = string.strip(path) + '/src'
            for file in os.listdir(path):
                if string.lower(str(file)) == string.lower(component) and os.path.isdir(os.path.join(path,component)):
                    if os.path.exists(os.path.join(path,component,'bin',string.lower(component)) ):
                        componentPath = os.path.join(path,component,'bin')
        if componentPath != '':
            return componentPath
        else:
            return False

    ''' find component source directory
        component - component name
        return    - the component directory in src
    '''
    def find_component_src(self,component):
        componentPath = []
        for path in self.workspace_paths:
            path = string.strip(path) + '/src'
            for file in os.listdir(path):
                if str(file) == component and os.path.isdir(os.path.join(path,component)):
                    componentPath.append(os.path.join(path,component))
        if len(componentPath) == 0:
            return False
        else:
            return componentPath

    ''' search and return a dictionary of file paths given component name and file name
        if duplicate components are found, first to found is chosen
        component - name of component as string
        files     - list of files to search for; if files[0]=* all files are returned
        return    - list of tuple of filnames and path
    '''
    def search_for_file(self,component,sfiles):
        filedict = []
        allfiledict = []
        #aa = [('testcomp1.cdsl', '/home/nithin/tmp/rc/rc_ws/src/testcomp1/testcomp1.cdsl'), ('README.md', '/home/nithin/tmp/rc/rc_ws/src/testcomp1/README.md')]
        path = self.find_component_src(component)
        #return aa
        componentPath = path[0]

        for sfile in sfiles:
            for root, dirs, files in os.walk(componentPath):                
                for tfile in files:
                    tmptuple = (tfile,os.path.join(root, tfile))
                    allfiledict.append(tmptuple)
                if sfile in files:
                    tmptuple = (sfile,os.path.join(root, sfile))
                    filedict.append(tmptuple)

        if sfile[0] == '*' :
            return allfiledict
        else:
            return filedict

    '''check if the given path is inside a workspace'''
    def find_workspace(self,path):
        for ws in self.workspace_paths:
            if path[:len(ws)] == ws:
                return ws
        return False

    ''' return all components paths given workspaces'''
    def list_packages(self,ws_paths):
        components = []
        for ws in ws_paths:
            if self.find_workspace(ws)==False:
                continue
            srcpath = ws + '/src'
            for component in os.listdir(srcpath):
                if os.path.isdir(os.path.join(srcpath, component))==False:
                    continue
                components.append(os.path.join(srcpath, component))
        return components


    def get_components_in_dir(self, initial_path):
        components_parents_dirs = defaultdict(list)
        for subdir, dirs, files in os.walk(initial_path):
            if 'CMakeLists.txt' in files and 'etc' in dirs \
                    and not subdir.startswith('test_')\
                    and '.local/share/Trash' not in subdir:
                for file in files:
                    if file.endswith(".cdsl"):
                        filepath= os.path.join(subdir,file)
                        components_parents_dirs[subdir].append(filepath)
                        print(f'Found {filepath}')
        return components_parents_dirs

    def find_robocomp_workspaces(self, initial_path):
        def common_prefix(path_list):
            result = {}
            to_ignore = ""
            "Given a list of pathnames, returns the longest common leading component"

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
        components_parents_dirs = self.get_components_in_dir(initial_path)

        #
        best_guess = common_prefix(list(components_parents_dirs.keys()))
        new_workspaces = []
        print(f"Found {len(best_guess)} possibles components in {colored(initial_path, 'green')}")
        print(f"======")
        ignored = []
        for path in sorted(best_guess, key=lambda x:len(x.split('/'))*best_guess[x], reverse=True):
            print(len(path.split('/'))*best_guess[path])
            end = False
            next = False
            for parent in new_workspaces:
                if path.startswith(parent):
                    print(f"{colored(path, 'grey')} ignored because you already selected parent {colored(parent, 'grey')}")
                    next = True
                    break
            for parent in ignored:
                if path.startswith(parent):
                    print(f"{colored(path, 'grey')} ignored because you decided to ignore subdirs of {colored(parent, 'grey')}")
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
                                  completer=FuzzyCompleter(MyCustomCompleter(path)),
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
                    end = True
                else:
                    print('Invalid response (y/n/i/e)')
            if end:
                break
        self.workspace_paths = new_workspaces
        self.components = list(filter(lambda x: any(x.startswith(workspace)for workspace in self.workspace_paths), components_parents_dirs))

        self.save_attr(self.workspace_paths, 'workspace_paths')
        self.save_attr(self.components, 'components')

    def save_attr(self, attr, filename):
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


    def load_attr(self, filename):
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
            pass

