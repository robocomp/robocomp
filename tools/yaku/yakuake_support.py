#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  -----------------------
#  -----  rcmanager  -----
#  -----------------------
#  An ICE component manager.
#
#    Copyright (C) 2009-2015 by RoboLab - University of Extremadura
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#

#
# CODE BEGINS
#

# Please refer to the following link for a nice documentation of the qdbus functions
# http://urfoex.blogspot.com/2016/08/qdbus-create-tabs-in-yakuake-and-pidgin.html

import subprocess
import shlex
import dbus
from time import sleep
from collections import OrderedDict

shell_code="""
#create new session for %s
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#get id of open session
sess0=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#run command on active session
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand " cd %s"
#run command on active session
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "%s"
#change the name of session
qdbus org.kde.yakuake /yakuake/tabs_by_name org.kde.yakuake.setTabTitle $sess0 "%s"
"""


def get_command_return(command):
    proc = subprocess.Popen(shlex.split(command), stderr=subprocess.PIPE,
                            stdout=subprocess.PIPE, shell=False)
    (result, error) = proc.communicate()
    return result, error

def get_last_child(pid):
    last_pid = pid
    while(True):
        child_pid,_ = get_command_return("pgrep -P %s"%last_pid)
        if not child_pid:
            return last_pid
        else:
            last_pid = int(child_pid)

class YakuakeTab:
    def __init__(self):
        self.session_id = -1
        self.session_id = -1
        self.terminal_id = -1
        self.name = ""
        self.last_command = ""
        self.tmp_path = ""
        self.current_directory = ""

def load_open_tabs_info():
    all_tabs = OrderedDict()
    bus = dbus.SessionBus()
    sessions = bus.get_object('org.kde.yakuake', '/yakuake/sessions')
    session_ids_text =sessions.sessionIdList()
    tabs =  bus.get_object('org.kde.yakuake', '/yakuake/tabs_by_name')
    session_ids = map(int, session_ids_text.split(','))
    for session_id in session_ids:
        yakuake_tab = YakuakeTab()
        yakuake_tab.session_id = session_id
        yakuake_tab.name = tabs.tabTitle(session_id)
        if "Shell" in yakuake_tab.name:
            continue
        yakuake_tab.terminal_id = sessions.terminalIdsForSessionId(session_id)

        konsole_session = bus.get_object('org.kde.yakuake','/Sessions/%s'%(session_id+1),'org.kde.konsole.Session')
        foreground_pid = konsole_session.foregroundProcessId()
        session_pid = konsole_session.processId()
        # Not executing the command
        if foreground_pid == session_pid:
            yakuake_tab.tmp_path = "/tmp/manager_commands_%d.txt" % session_id
            sessions.runCommandInTerminal(int(yakuake_tab.terminal_id),
                                          "pwd > %s" % yakuake_tab.tmp_path)
            sessions.runCommandInTerminal(int(yakuake_tab.terminal_id),
                                          "history 3 | head -1 | cut -d' ' -f4- >> %s" % yakuake_tab.tmp_path)
            sleep(0.5)

            for retrie in range(5):
                try:
                    with open(yakuake_tab.tmp_path) as command_file:
                        file_lines = command_file.read().splitlines()
                        if len(file_lines) > 0:
                            yakuake_tab.current_directory = file_lines[0]
                        if len(file_lines) > 1:
                            yakuake_tab.last_command = file_lines[1]

                        # print("Tab name: %s \n\tId: %d \n\tLast Command: %s \n\tCurrent dir:%s\n" % (yakuake_tab.name,yakuake_tab.session_id, yakuake_tab.last_command, yakuake_tab.current_directory))
                        break
                except:
                    if retrie == 4:
                        print(
                                    "Could not get command from \"%s\" tab. Check that you finished the process that tab." % yakuake_tab.name)
        else:
            child_pid = get_last_child(foreground_pid)
            pwdx_command = "pwdx %s" % child_pid
            pwd, error = get_command_return(pwdx_command)
            pwd = pwd.split()[1].strip()
            yakuake_tab.current_directory = pwd
            ps_command = "ps -p %s -o cmd h" % child_pid
            command, error = get_command_return(ps_command)
            command = command.strip()
            yakuake_tab.last_command = command

        all_tabs[yakuake_tab.name] = yakuake_tab
        # finishing writting temp files


    return all_tabs

def create_xml(tabs):
    from lxml import etree
    from xml.dom import minidom
    import xml.etree.ElementTree as ET
    root = etree.Element("rcmanager")


    generalInformation = etree.Element("generalInformation")
    editor = etree.Element("editor", path="gedit", dock="false" )
    timeouts = etree.Element("timeouts", fixed="1000.0", blink="300.0" )
    clicks = etree.Element("clicks", switch="2.0", interval="400.0" )
    graph = etree.Element("graph", alpha="80.0", active="true", scale="200.0" )
    graphTiming = etree.Element("graphTiming", idletime="1000.0", focustime="500.0", fasttime="10.0", fastperiod="2000.0" )
    simulation = etree.Element("simulation", hookes="0.07", springlength="0.5", friction="0.4", step="0.5", fieldforce="20000.0" )
    root.append(generalInformation)
    generalInformation.append(editor)
    generalInformation.append(timeouts)
    generalInformation.append(clicks)
    generalInformation.append(graph)
    generalInformation.append(graphTiming)
    generalInformation.append(simulation)
    for tab in tabs:
        node = etree.Element("node", alias=tab.name, endpoint="")
        dependence = etree.Element("dependence", alias="" )
        workingDir = etree.Element("workingDir", path=tab.current_directory )
        upCommand = etree.Element("upCommand", command="rcremote localhost %s" % tab.last_command )
        downCommand = etree.Element("downCommand", command="" )
        configFile = etree.Element("configFile", path="" )
        xpos = etree.Element("xpos", value="31.734059895" )
        ypos = etree.Element("ypos", value="-187.269590989" )
        radius = etree.Element("radius", value="10.0" )
        color = etree.Element("color", value="#AAAAAA" )
        node.append(dependence)
        node.append(workingDir)
        node.append(upCommand)
        node.append(downCommand)
        node.append(configFile)
        node.append(xpos)
        node.append(ypos)
        node.append(radius)
        node.append(color)
        root.append(node)
    # print(etree.tostring(root, pretty_print=True))
    xmlstr = minidom.parseString(ET.tostring(root)).toprettyxml(indent="   ")
    with open("./new_deplyment.xml", "w") as f:
        f.write(xmlstr)

def create_yakuake_start_shell_script(tabs_to_restore=None):
    if tabs_to_restore is None:
        tabs_to_restore = load_open_tabs_info()
    shell_script_content = ""
    for tab in tabs_to_restore.values():
        if tab.last_command != "":
            shell_script_content+=shell_code%(tab.name, tab.current_directory, tab.last_command, tab.name)
    with open("./new_deplyment.sh", "w") as f:
        f.write(shell_script_content)
    print("Script saved in ./new_deployment.sh")


class ProcessHandler():
    def __init__(self):
        try:
            proc = subprocess.Popen(shlex.split("yakuake"), shell=False)
        except WindowsError:
            print "Yakuake is not supported on windows systems"
        self.tabTitleToSessionId = dict()
        self.sessionIdToTabTitle = dict()

    def start_process_in_new_session(self, tabTitle=None, command=None):
        try:
            ret, sessionId = self.add_session(tabTitle)
            proc = subprocess.Popen(
                shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommandInTerminal ' \
                            + str(sessionId) + ' "' + command + '"'), shell=False)
            sessionProcessId = self.get_session_process_id(tabTitle)
            foregroundProcessId = sessionProcessId

            while foregroundProcessId == sessionProcessId:
                foregroundProcessId = self.get_foreground_process_id(tabTitle)
        except Exception, e:
            raise e
        return (tabTitle, foregroundProcessId)

    def start_process_in_existing_session(self, tabTitle=None, command=None):
        if tabTitle in self.tabTitleToSessionId:
            sessionId = int(self.tabTitleToSessionId[tabTitle])
            proc = subprocess.Popen(
                shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.isSessionClosable ' \
                            + str(sessionId)), stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=False)
            (isSessionExisting, error) = proc.communicate()

            if isSessionExisting == "true\n":
                try:
                    proc = subprocess.Popen(
                        shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommandInTerminal ' \
                                    + str(sessionId) + ' "' + command + '"'), shell=False)
                    sessionProcessId = self.get_session_process_id(tabTitle)
                    foregroundProcessId = sessionProcessId
                    while foregroundProcessId == sessionProcessId:
                        foregroundProcessId = self.get_foreground_process_id(tabTitle)
                except Exception, e:
                    raise e
                return (tabTitle, foregroundProcessId)
        ret = self.start_process_in_new_session(tabTitle, command)
        return ret

    def stop_process_in_session(self, tabTitle=None):
        try:
            processId = self.get_foreground_process_id(tabTitle)
            proc = subprocess.Popen(shlex.split('kill -9 ' + str(processId)), shell=False)
        except Exception, e:
            raise e

    def add_session(self, tabTitle):
        try:
            proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession'), \
                                    stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=False)
            (sessionId, error) = proc.communicate()
            proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /yakuake/tabs_by_name org.kde.yakuake.setTabTitle ' + \
                                                str(sessionId) + ' "' + tabTitle + '"'), shell=False)
            self.tabTitleToSessionId[tabTitle] = sessionId
            self.sessionIdToTabTitle[sessionId] = tabTitle
        except Exception, e:
            raise e
        return (tabTitle, sessionId)

    def close_session(self, tabTitle):
        try:
            proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.removeSession ' \
                                                + str(self.tabTitleToSessionId[tabTitle])), shell=False)
        except Exception, e:
            raise e

    def get_foreground_process_id(self, tabTitle):
        sessionIdPlusOne = int(self.tabTitleToSessionId[tabTitle]) + 1
        proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /Sessions/' + str(
            sessionIdPlusOne) + ' org.kde.konsole.Session.foregroundProcessId'), stderr=subprocess.PIPE,
                                stdout=subprocess.PIPE, shell=False)
        (processId, error) = proc.communicate()
        return processId

    def get_session_process_id(self, tabTitle):
        sessionIdPlusOne = int(self.tabTitleToSessionId[tabTitle]) + 1
        proc = subprocess.Popen(shlex.split('qdbus org.kde.yakuake /Sessions/' + str(
            sessionIdPlusOne) + ' org.kde.konsole.Session.processId'), stderr=subprocess.PIPE,
                                stdout=subprocess.PIPE, shell=False)
        (processId, error) = proc.communicate()
        return processId

if __name__ == '__main__':
    load_open_tabs_info()
    # processHandler = ProcessHandler()


