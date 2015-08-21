#!/usr/bin/python

import os
import string
import shutil

''' Basic module which defines the workspace class
'''

class Workspace:
    
    workspace_paths = []
    numOfws = 0
    robocomp_dir = ''

    ''' constructor'''
    def __init__(self):
        self.update_pathlist()
    
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
        except Exception, e:
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
                raise RuntimeError("couldnt find toplevel cmake, make sure ROBOCOMP is properly installed\n")
            
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
            raise RuntimeError("Sorry, you cant create workspace inside an existing one \n")

        try:
            os.system('touch {0}/{1}'.format(ws_path,str(".rc_workspace")))
            dirs = ["src","build","install"]
            for dir in dirs:
                dir_path = os.path.join(ws_path,dir)
                if not os.path.exists(dir_path):
                    os.makedirs(dir_path)
            os.system('touch {0}/install/{1}'.format(ws_path,str(".rc_install")))
        except Exception as createEx:
            raise RuntimeError("couldnt create files/folders in current directory: \n %s " % (str(createEx)))

        pathstr = str(os.path.abspath(ws_path))
        
        #copy all files in the current dir into src dir
        print("copying all files in workspce directory into src/ directory\n")
        for file in os.listdir(pathstr):
            if file in dirs or file in [".rc_workspace"] : continue
            try:
                os.system("mv ./" + file + " ./src")
            except Exception as copyEx:
                print("couldnt copy all files\n")

        create_toplevel(ws_path+'/src')
        self.register_workspace(pathstr)

    ''' find the directory containing component exexutable'''
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

    ''' find component soruce directory
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

    ''' search and return a dictionry of file paths given component name and file name
        if duplicate components are found first to found is chosen
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

workspace = Workspace()
