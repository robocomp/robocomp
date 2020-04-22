#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse

import dbus
import os
import shlex
import subprocess


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


def rename_current_tab(name = None, append=False):
    bus = dbus.SessionBus()

    active_session = bus.get_object("org.kde.yakuake", "/yakuake/sessions", "org.kde.yakuake").activeSessionId()

    konsole_session = bus.get_object('org.kde.yakuake', '/Sessions/%s' % (active_session + 1), 'org.kde.konsole.Session')
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


    bus.get_object("org.kde.yakuake", "/yakuake/tabs", "org.kde.yakuake").setTabTitle(active_session, final_name)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--append', action='store_true', dest="append",  help="append the given name to the directory name for the Tab title")
    parser.add_argument('name', nargs='?', help="Alternative name for the tab")
    args = parser.parse_args()
    rename_current_tab(args.name, args.append)
