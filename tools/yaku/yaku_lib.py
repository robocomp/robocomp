#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from collections import OrderedDict
from time import sleep

import dbus
import os
import shlex
import subprocess


SHELL_CODE="""
#create new session for %s
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#get id of open session
sess0=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#run command on active session
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand " cd %s"
#run command on active session
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "%s"
#change the name of session
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess0 "%s"
"""

def get_command_return(command):
    proc = subprocess.Popen(shlex.split(command), stderr=subprocess.PIPE,
                            stdout=subprocess.PIPE, shell=False)
    (result, error) = proc.communicate()
    return result, error


def get_last_child(pid):
    last_pid = pid
    while (True):
        child_pid, _ = get_command_return("pgrep -P %s" % last_pid)
        if not child_pid:
            return last_pid
        else:
            last_pid = int(child_pid)

class YakuakeTab:
    def __init__(self):
        self.session_id = -1
        self.terminal_id = -1
        self.name = ""
        self.last_command = ""
        self.tmp_path = ""
        self.current_directory = ""

class Yaku:
    def __init__(self):
        self.bus = dbus.SessionBus()
        self.sessions = self.bus.get_object("org.kde.yakuake", "/yakuake/sessions", "org.kde.yakuake")
        self.active_session = self.sessions.activeSessionId()
        self.tabs = OrderedDict()


    def rename_current_tab(self, name=None, append=False):

        konsole_session = self.bus.get_object('org.kde.yakuake', '/Sessions/%s' % (self.active_session + 1), 'org.kde.konsole.Session')

        foreground_pid = konsole_session.foregroundProcessId()
        session_pid = konsole_session.processId()

        child_pid = get_last_child(foreground_pid)
        pwdx_command = "pwdx %s" % child_pid
        pwd, error = get_command_return(pwdx_command)
        pwd = pwd.split()[1].strip()

        dir_name = os.path.basename(os.path.normpath(pwd))
        if name is None:
            final_name = dir_name.decode("utf-8")
        else:
            if append:
                final_name = "%s - %s" % (dir_name.decode("utf-8"), name)
            else:
                final_name = name
        self.bus.get_object("org.kde.yakuake", "/yakuake/tabs", "org.kde.yakuake").setTabTitle(self.active_session, final_name)
    # @staticmethod
    # def rename_all_tabs(name=None, append=False):

    def load_open_tabs_info(self):
        session_ids_text = self.sessions.sessionIdList()
        open_tabs = self.bus.get_object('org.kde.yakuake', '/yakuake/tabs')
        session_ids = map(int, session_ids_text.split(','))
        for session_id in sorted(session_ids):
            yakuake_tab = YakuakeTab()
            yakuake_tab.session_id = session_id
            yakuake_tab.name = open_tabs.tabTitle(session_id)
            if "Shell" in yakuake_tab.name:
                continue
            yakuake_tab.terminal_id = self.sessions.terminalIdsForSessionId(session_id)

            konsole_session = self.bus.get_object('org.kde.yakuake', '/Sessions/%s' % (session_id + 1),
                                             'org.kde.konsole.Session')
            foreground_pid = konsole_session.foregroundProcessId()
            session_pid = konsole_session.processId()
            # Not executing the command
            if foreground_pid == session_pid:
                yakuake_tab.tmp_path = "/tmp/yaku_commands_%d.txt" % session_id
                self.sessions.runCommandInTerminal(int(yakuake_tab.terminal_id),
                                              "pwd > %s" % yakuake_tab.tmp_path)
                self.sessions.runCommandInTerminal(int(yakuake_tab.terminal_id),
                                              "history 3 | head -1 | cut -d' ' -f4- >> %s" % yakuake_tab.tmp_path)
                sleep(0.5)

                for retrie in range(5):
                    try:
                        with open(yakuake_tab.tmp_path) as command_file:
                            file_lines = command_file.read().splitlines()
                            if len(file_lines) > 0:
                                yakuake_tab.current_directory = file_lines[0]
                            if len(file_lines) > 1 and "history 3" not in file_lines[1]:
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
                yakuake_tab.current_directory = pwd.decode("utf-8")
                ps_command = "ps -p %s -o cmd h" % child_pid
                command, error = get_command_return(ps_command)
                command = command.strip()
                yakuake_tab.last_command = command.decode("utf-8")

            self.tabs[yakuake_tab.name] = yakuake_tab
            # finishing writting temp files

    def create_yakuake_start_shell_script(self, tabs_to_restore=None):
        if tabs_to_restore is None:
            self.load_open_tabs_info()
            tabs_to_restore = self.tabs
        shell_script_content = ""
        for tab in tabs_to_restore.values():
            if tab.last_command != "" or tab.current_directory != "":
                shell_script_content += SHELL_CODE % (tab.name, tab.current_directory, tab.last_command, tab.name)
        with open("./new_deplyment.sh", "w") as f:
            f.write(shell_script_content)
        print("Script saved in ./new_deployment.sh")


if __name__ == '__main__':
    yaku = Yaku()
    yaku.create_yakuake_start_shell_script()
    print(yaku.tabs)

