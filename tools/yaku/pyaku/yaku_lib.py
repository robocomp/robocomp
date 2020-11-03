#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from collections import OrderedDict
from time import sleep

import dbus
import os
import shlex
import subprocess


TAB_SHELL_CODE = """
#create new tab for %s
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#get id of open session
sess0=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#run command on active session
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand " cd %s"
#run command on active session
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand "%s"
#change the title of session
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess0 "%s"
"""

TERMINA_SPLIT_SHELL_CODE = """
"""

def get_command_return(command):
    proc = subprocess.Popen(command, stderr=subprocess.PIPE,
                            stdout=subprocess.PIPE, shell=True, executable='/bin/bash')
    proc.wait()
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


class YakuakeDBus(object):
    __instance = None
    def __new__(cls):
        if YakuakeDBus.__instance is None:
            YakuakeDBus.__instance = object.__new__(cls)
        YakuakeDBus.__instance.s_dbus = dbus.SessionBus()
        YakuakeDBus.__instance.tabs = YakuakeDBus.__instance.s_dbus.get_object("org.kde.yakuake", "/yakuake/tabs", "org.kde.yakuake")
        YakuakeDBus.__instance.sessions = YakuakeDBus.__instance.s_dbus.get_object("org.kde.yakuake", "/yakuake/sessions", "org.kde.yakuake")
        YakuakeDBus.__instance.konsole_sessions = {}
        return YakuakeDBus.__instance

    def session(self, session_id: int):
        if session_id not in self.konsole_sessions:
            self.konsole_sessions[session_id] = self.s_dbus.get_object('org.kde.yakuake',
                                                                            '/Sessions/%s' % (session_id),
                                                                            'org.kde.konsole.Session')
        return self.konsole_sessions[session_id]


class KonsoleSession:
    def __init__(self, terminal_id):
        self.konsole_session_id = terminal_id+1
        self.terminal_id = terminal_id
        dbus_session = YakuakeDBus().session(self.konsole_session_id)
        self.foreground_pid = dbus_session.foregroundProcessId()
        self.session_pid = dbus_session.processId()
        # Not executing the command
        self.child_pid = get_last_child(self.foreground_pid)
        pwdx_command = "pwdx %s" % self.child_pid
        pwd, error = get_command_return(pwdx_command)
        pwd = pwd.split()[1].strip()
        self.current_directory = pwd.decode("utf-8")
        self.__last_command = ""

    def last_command(self):
        if self.foreground_pid == self.session_pid:
            self.__last_command = self.get_last_executed_command(self.terminal_id)
        else:
            ps_command = "ps -p %s -o cmd h" % self.child_pid
            command, error = get_command_return(ps_command)
            command = command.strip()
            self.__last_command = command.decode("utf-8")
        return self.__last_command

    def get_last_executed_command(self, terminal_id):

        tmp_path = "/tmp/yaku_commands_%d.txt" % terminal_id
        # WARNING: leading space character in echo command is imprtant to avoid this been saved in history
        # https://stackoverflow.com/questions/6475524/how-do-i-prevent-commands-from-showing-up-in-bash-history
        YakuakeDBus().sessions.runCommandInTerminal(int(terminal_id), f" echo !:0 !:* > {tmp_path}")
        sleep(0.1)

        for retrie in range(5):
            try:
                with open(tmp_path) as command_file:
                    file_lines = command_file.read().splitlines()
                    if len(file_lines) > 0:
                        return file_lines[0]
                    # print("Tab title: %s \n\tId: %d \n\tLast Command: %s \n\tCurrent dir:%s\n" % (yakuake_tab.title,yakuake_tab.session_id, yakuake_tab.last_command, yakuake_tab.current_directory))
                    break
            except:
                if retrie == 4:
                    print(
                        "Could not get command from terminal \"%s\"" % str(terminal_id))
        return ""

class YakuakeSessionStack:
    def __init__(self):
        self.__dbus = dbus.SessionBus()
        self.__dbus_sessions_obj = YakuakeDBus().sessions
        self.__active_session_id = -1
        self.__terminal_session_id = -1
        self.sessions = OrderedDict()
        self.open_tabs = YakuakeDBus().tabs

    @property
    def active_session_id(self)  -> int:
        self.__active_session_id = self.__dbus_sessions_obj.activeSessionId()
        return self.__active_session_id

    @property
    def active_terminal_id(self) -> int:
        self.__active_terminal_id = self.__dbus_sessions_obj.activeTerminalId()
        return self.__active_terminal_id

    def add_session(self):
        self.__dbus_sessions_obj.addSession()

    def add_session_quad(self):
        self.__dbus_sessions_obj.addSessionQuad()

    def add_session_two_horizontal(self):
        self.__dbus_sessions_obj.addSessionTwoHorizontal()

    def add_session_two_vertical(self):
        self.__dbus_sessions_obj.addSessionTwoVertical()


    def raise_session(self, session_id: int):
        self.__dbus_sessions_obj.raiseSession()

    def remove_session(self, session_id: int):
        self.__dbus_sessions_obj.removeSession()

    def remove_terminal(self, terminal_id: int):
        self.__dbus_sessions_obj.removeTerminal()


    def split_session_left_right(self, session_id: int):
        self.__dbus_sessions_obj.splitSessionLeftRight()

    def split_session_top_bottom(self, session_id: int):
        self.__dbus_sessions_obj.splitSessionTopBottom()

    def split_terminal_left_right(self, terminal_id: int):
        self.__dbus_sessions_obj.splitTerminalLeftRight()

    def split_terminal_top_bottom(self, terminal_id: int):
        self.__dbus_sessions_obj.splitTerminalTopBottom()


    def try_grow_terminal_right(self, terminal_id: int, pixels: int = 10):
        self.__dbus_sessions_obj.tryGrowTerminalRight(terminal_id, pixels)


    def try_grow_terminal_left(self, terminal_id: int, pixels: int = 10):
        self.__dbus_sessions_obj.tryGrowTerminalLeft(terminal_id, pixels)


    def try_grow_terminal_top(self, terminal_id: int, pixels: int = 10):
        self.__dbus_sessions_obj.tryGrowTerminalTop(terminal_id, pixels)


    def try_grow_terminal_bottom(self, terminal_id: int, pixels: int = 10):
        self.__dbus_sessions_obj.tryGrowTerminalBottom(terminal_id, pixels)

    def session_id_list(self) -> list:
        session_ids_text = self.__dbus_sessions_obj.sessionIdList()
        session_ids = map(int, session_ids_text.split(','))
        return list(session_ids)

    def terminal_id_list(self) -> list:
        terminal_ids_text = self.__dbus_sessions_obj.terminalIdList()
        terminal_ids = map(int, terminal_ids_text.split(','))
        return list(terminal_ids)


    def terminal_ids_for_session_id(self, session_id: int) -> int:
        terminal_ids_text = self.__dbus_sessions_obj.terminalIdsForSessionId(session_id)
        terminal_ids = map(int, terminal_ids_text.split(','))
        return list(terminal_ids)

    def session_id_for_terminal_id(self, terminal_id: int) -> int:
        return int(self.__dbus_sessions_obj.sessionIdForTerminalId(terminal_id))


    def run_command(self, command: str):
        self.__dbus_sessions_obj.runCommand(command)

    def run_command_in_terminal(self, terminal_id: int, command: str):
        self.__dbus_sessions_obj.runCommandInTerminal(terminal_id, command)

    def is_session_closable(self, session_id: int) -> bool:
        return self.__dbus_sessions_obj.isSessionClosable(session_id)


    def set_session_closable(self, session_id: int, closable: bool):
        self.__dbus_sessions_obj.setSessionClosable(session_id, closable)

    def has_unclosable_sessions(self) -> bool :
        return self.__dbus_sessions_obj.hasUnclosableSessions()

    def is_session_keyboard_input_enabled(self, session_id: int)  -> bool:
        return self.__dbus_sessions_obj.isSessionKeyboardInputEnabled(session_id)

    def set_session_keyboard_input_enabled(self, session_id: int, enabled: bool):
        self.__dbus_sessions_obj.setSessionKeyboardInputEnabled(session_id, enabled)

    def is_terminal_keyboard_input_enabled(self, terminal_id: int)  -> bool:
        return self.__dbus_sessions_obj.isTerminalKeyboardInputEnabled(terminal_id)

    def set_terminal_keyboard_input_enabled(self, terminal_id: int, enabled: bool):
        self.__dbus_sessions_obj.setTerminalKeyboardInputEnabled(terminal_id, enabled)

    def has_terminals_with_keyboard_input_enabled(self, session_id: int) -> bool:
        return self.__dbus_sessions_obj.hasTerminalsWithKeyboardInputEnabled(session_id)

    def has_terminals_with_keyboard_input_disabled(self, session_id: int) -> bool:
        return self.__dbus_sessions_obj.hasTerminalsWithKeyboardInputDisabled(session_id)

    def is_session_monitor_activity_enabled(self, session_id: int)  -> bool:
        return self.__dbus_sessions_obj.isSessionMonitorActivityEnabled(session_id)

    def set_session_monitor_activity_enabled(self, session_id: int, enabled: bool):
        self.__dbus_sessions_obj.setSessionMonitorActivityEnabled(session_id, enabled)

    def is_terminal_monitor_activity_enabled(self, terminal_id: int)  -> bool:
        return self.__dbus_sessions_obj.isTerminalMonitorActivityEnabled(terminal_id)

    def set_terminal_monitor_activity_enabled(self, terminal_id: int, enabled: bool):
        self.__dbus_sessions_obj.setTerminalMonitorActivityEnabled(terminal_id, enabled)

    def has_terminals_with_monitor_activity_enabled(self, session_id: int) -> bool:
        return self.__dbus_sessions_obj.hasTerminalsWithMonitorActivityEnabled(session_id)

    def has_terminals_with_monitor_activity_disabled(self, session_id) -> bool:
        return self.__dbus_sessions_obj.hasTerminalsWithMonitorActivityDisabled(session_id)

    def is_session_monitor_silence_enabled(self, session_id: int)  -> bool:
        return self.__dbus_sessions_obj.isSessionMonitorSilenceEnabled(session_id)

    def set_session_monitor_silence_enabled(self, session_id: int, enabled: bool):
        self.__dbus_sessions_obj.setSessionMonitorSilenceEnabled(session_id, enabled)

    def is_terminal_monitor_silence_enabled(self, terminal_id: int)  -> bool:
        return self.__dbus_sessions_obj.isTerminalMonitorSilenceEnabled(terminal_id)

    def set_terminal_monitor_silence_enabled(self, terminal_id: int, enabled: bool):
        self.__dbus_sessions_obj.setTerminalMonitorSilenceEnabled(terminal_id, enabled)

    def has_terminals_with_monitor_silence_enabled(self, session_id) -> bool:
        return self.__dbus_sessions_obj.hasTerminalsWithMonitorSilenceEnabled(session_id)

    def has_terminals_with_monitor_silence_disabled(self, session_id: int) -> bool:
        return self.__dbus_sessions_obj.hasTerminalsWithMonitorSilenceDisabled(session_id)

class YakuakeTabStack:
    def __init__(self):
        self.__dbus = dbus.SessionBus()
        self.open_tabs = YakuakeDBus().tabs
        self.session_stack = YakuakeSessionStack()
        self.tabs_by_name = OrderedDict()
        self.tabs_by_index = OrderedDict()
        self.tabs_by_session = OrderedDict()

    def load_open_tabs_info(self):
        next_index = 0
        session_id = None
        while session_id != -1:
            session_id = self.session_at_tab(next_index)
            if session_id != -1:
                tab = YakuakeTab(next_index, session_id)
                self.tabs_by_index[next_index] = tab
                self.tabs_by_session[session_id] = tab
                next_index += 1

        for index, tab in self.tabs_by_index.items():
            self.load_tab_info(tab)

    def session_at_tab(self, index: int) -> int:
        return int(self.__dbus.get_object("org.kde.yakuake", "/yakuake/tabs", "org.kde.yakuake").sessionAtTab(index))

    def tab_title(self, session_id):
        return self.__dbus.get_object("org.kde.yakuake", "/yakuake/tabs", "org.kde.yakuake").tabTitle(session_id)

    def terminal_ids_for_index(self, index):
        tab = self.tabs_by_index[index]
        session_id = tab.yakuake_session_id
        return self.session_stack.terminal_ids_for_session_id(session_id)


    def load_tab_info(self, tab):
        tab.title = self.tab_title(tab.yakuake_session_id)
        tab.terminal_ids = self.terminal_ids_for_index(tab.index)
        for terminal_id in tab.terminal_ids:
            konsole_terminal = KonsoleSession(terminal_id)
            tab.terminals[terminal_id] = konsole_terminal
        same_name_count = 0
        new_name = tab.title
        while new_name in self.tabs_by_name:
            same_name_count += 1
            new_name = tab.title + '_' + str(same_name_count)
        self.tabs_by_name[new_name] = tab

    def rename_tab_by_index(self, index: int, title: str):
        YakuakeDBus().tabs.setTabTitle(index, title)

    def rename_all_tabs(self, name=None, append=False):
        self.load_open_tabs_info()
        for index, tab in self.tabs_by_index.items():
            first_terminal = sorted(tab.terminals.items())[0][1]
            pwd = first_terminal.current_directory
            dir_name = os.path.basename(os.path.normpath(pwd))
            if name is None:
                final_name = dir_name
            else:
                if append:
                    final_name = "%s - %s" % (dir_name.decode("utf-8"), name)
                else:
                    final_name = name
            if len(tab.terminals)>1:
                final_name += "_+"+str(len(tab.terminals)-1)
            final_name = '\"%s\"'%final_name
            YakuakeDBus().tabs.setTabTitle(tab.yakuake_session_id, final_name)


class YakuakeTab:
    def __init__(self, index, session_id):
        self.index = index
        self.yakuake_session_id = session_id
        self.terminal_ids = []
        self.terminals = {}
        self.title = ""
        self.last_command = ""
        self.tmp_path = ""
        self.__current_directory = ""

    @property
    def current_directory(self):
        if len(self.terminals) > 0:
            lower_terminal_id = sorted(self.terminals)[0]
            self.__current_directory = self.terminals[lower_terminal_id].current_directory
        return self.__current_directory


class Yaku:
    def __init__(self):
        self.__dbus = dbus.SessionBus()
        self.session_stack = YakuakeSessionStack()
        self.tabs_stack = YakuakeTabStack()
        if len(self.tabs_stack.tabs_by_index) == 0:
            self.tabs_stack.load_open_tabs_info()

    def toggle_window_state(self):
        self.__dbus.toggleWindowState()


    def rename_tab(self, name=None, append=False, session_id=None):
        if session_id is None:
            session_id = self.session_stack.active_session_id
        pwd = self.tabs_stack.tabs_by_session[session_id].current_directory
        dir_name = os.path.basename(os.path.normpath(pwd))
        if name is None:
            final_name = dir_name
        else:
            if append:
                final_name = "%s - %s" % (dir_name.decode("utf-8"), name)
            else:
                final_name = name
        if len(self.tabs_stack.tabs_by_session[session_id].terminals) > 1:
            final_name += " +"
        YakuakeDBus().tabs.setTabTitle(session_id, final_name)

    def rename_current_tab(self, name=None, append=False):
        self.rename_tab(name, append)


    # @staticmethod
    # def rename_all_tabs(title=None, append=False):



    def create_yakuake_start_shell_script(self, tabs_to_restore=None):
        if tabs_to_restore is None:
            if len(self.tabs_stack.tabs_by_index) == 0:
                self.tabs_stack.load_open_tabs_info()
            tabs_to_restore = self.tabs_stack.tabs_by_name
        shell_script_content = ""
        for tab in tabs_to_restore.values():
            for terminal in tab.terminals.values():
                if (terminal.last_command != "" or terminal.current_directory != "") and ("Shell" not in tab.title and "Consola" not in tab.title):
                    tab_title = tab.title
                    if len(tab.terminals) > 1:
                        tab_title += str(terminal.konsole_session_id)
                    shell_script_content += TAB_SHELL_CODE % (tab_title, terminal.current_directory, terminal.last_command, tab_title)
        with open("./new_deplyment.sh", "w") as f:
            f.write(shell_script_content)
        print("Script saved in ./new_deployment.sh")

    def rename_all_tabs(self, name=None, append=False):
        self.tabs_stack.rename_all_tabs(name, append)


if __name__ == '__main__':
    yaku = Yaku()
    yaku.rename_all_tabs()
    # print(yaku.tabs_by_name)

