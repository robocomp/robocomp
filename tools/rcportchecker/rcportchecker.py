#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
#  ------------------------
#  ----- rcportchecker -----
#  ------------------------
#
#    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
#
#

import argparse
import os
import re
import sys


class BColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class MyParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write((BColors.FAIL + 'error: %s\n' + BColors.ENDC) % message)
        self.print_help()
        sys.exit(2)


class RCPortChecker:
    def __init__(self, verbose=False, path=None):
        self.interfaces_ports = {}
        self.ports_for_interfaces = {}
        self.debug = verbose
        if path is not None:
            self.robocomp_path = path
        else:
            self.robocomp_path = self.default_dir()
        self.find_and_parse_config_files()

    def clear(self):
        self.interfaces_ports = {}
        self.ports_for_interfaces = {}

    def default_dir(self):
        default_robocomp_path = os.path.join(os.path.expanduser("~"), "robocomp")
        if self.debug:
            print('Trying default directory (%s)' % (default_robocomp_path))
        if os.path.exists(default_robocomp_path):
            if self.debug:
                print("Default Robocomp (%s) exists " % (default_robocomp_path))
            if os.path.isdir(default_robocomp_path):
                if self.debug:
                    print("Default Robocomp directory (%s) is a directory " % (default_robocomp_path))
                if not os.listdir(default_robocomp_path):
                    print((
                                  BColors.FAIL + "Default Robocomp directory (%s) exists but it's empty. Exiting!" + BColors.ENDC) % (
                              default_robocomp_path))
                    sys.exit()
                else:
                    return default_robocomp_path
            else:
                print((
                              BColors.FAIL + "Default Robocomp directory (%s) exists but it's not a directory. Exiting!" + BColors.ENDC) % (
                          default_robocomp_path))
                sys.exit()
        else:
            print((BColors.FAIL + "Default Robocomp directory (%s) doesn't exists. Exiting!" + BColors.ENDC) % (
                default_robocomp_path))
            sys.exit()

    def print_interfaces_and_paths(self, interfaces, interface_filter):
        for interface, paths in interfaces.items():
            if interface_filter is not None and interface_filter not in interface.lower():
                continue
            print("\t%s" % (interface))
            for path in paths:
                print("\t\t%s" % (path))

    def print_port_listing(self, all=False, lower=False, interface_filter=None):
        for port, interfaces in sorted(self.ports_for_interfaces.items()):
            to_show = True
            if not all and len(interfaces) < 2:
                to_show = False
            if lower and port > 10000:
                to_show = False
            if to_show:
                print("In port %d\t" % (port))
                self.print_interfaces_and_paths(interfaces, interface_filter)

    def print_interface_listing(self, all=False, lower=False, interface_filter=None):
        for interface, ports in sorted(self.interfaces_ports.items()):
            if interface_filter is not None and interface_filter.lower() not in interface.lower():
                continue
            to_show = True
            if not all and len(ports) < 2:
                to_show = False
            if to_show:
                print("%s" % (interface))
                for port, paths in ports.items():
                    if lower and port > 10000:
                        to_show = False
                    else:
                        to_show = True
                    if to_show:
                        print("\t%d" % (port))
                        for path in paths:
                            print("\t\t%s" % (path))

    def search_interface_port_by_name(self, interface_name):
        ports = self.search_interface_ports_by_name(interface_name)
        if ports is not None:
            port_key = ports.keys[0]
            return port_key, ports[port_key]
        else:
            return None

    def search_interface_ports_by_name(self, interface_name):
        for interface, ports in sorted(self.interfaces_ports.items()):
            if interface_name is not None and interface_name.lower() != interface.lower():
                continue
            return ports
        return None

    def search_interface_by_port(self, port):
        if port in self.ports_for_interfaces.keys():
            interfaces = self.ports_for_interfaces[port]
            interface_key = list(interfaces.keys())[0]
            return interface_key, interfaces[interface_key]
        else:
            return None, None

    def find_and_parse_config_files(self, paths=None):
        for root, dirs, files in os.walk(self.robocomp_path, topdown=False):
            for name in files:
                file_name, file_extension = os.path.splitext(name)
                if ("config" in name and file_extension == '') or (".conf" in file_extension) and "etc" in root:
                    fullpath = os.path.join(root, name)
                    self.parse_config_file(fullpath)
        if self.debug:
            print("\n---\n")

    def parse_config_file(self, fullpath):
        with open(fullpath, 'r') as fin:
            for line in fin:
                if ".Endpoints" in line and "cog.outl" not in line:
                    if self.debug:
                        print("Looking into file %s" % (fullpath))
                    extracted = re.findall(r'(.*)\.Endpoints\s*=.*-p\s?(\d+)', line)
                    if extracted:
                        if self.debug:
                            print("\tfound endpoint %s" % (line))
                        (interface_name, port) = extracted[0]
                        port = int(port)
                        if interface_name in self.interfaces_ports:
                            if port not in self.interfaces_ports[interface_name].keys():
                                self.interfaces_ports[interface_name][port] = [fullpath]
                            else:
                                self.interfaces_ports[interface_name][port].append(fullpath)
                        else:
                            self.interfaces_ports[interface_name] = {}
                            self.interfaces_ports[interface_name][port] = [fullpath]

                        if port in self.ports_for_interfaces:
                            if interface_name not in self.ports_for_interfaces[port]:
                                self.ports_for_interfaces[port][interface_name] = [fullpath]
                            else:
                                self.ports_for_interfaces[port][interface_name].append(fullpath)
                        else:
                            self.ports_for_interfaces[port] = {}
                            self.ports_for_interfaces[port][interface_name] = [fullpath]
                    else:
                        print((BColors.WARNING + "Interface without port? %s" + BColors.ENDC) % (line))

    def print_port_info(self, port, all=False, lower=False, interface_filter=None):
        if port in self.ports_for_interfaces:
            interfaces = self.ports_for_interfaces[port]
            to_show = True
            if not all and len(interfaces) < 2:
                to_show = False
            if lower and port > 10000:
                to_show = False
            if to_show:
                print("In port %d\t" % (port))
                self.print_interfaces_and_paths(interfaces, interface_filter)
        else:
            print((BColors.OKBLUE + "Port \'%d\' not found" + BColors.ENDC) % (port))


def main():
    parser = MyParser(description='Application to look for existing configured interfaces ports on components')
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                        action="store_true")
    parser.add_argument("-p", "--port", help="List only the selected port information",
                        type=int)
    # parser.add_argument("-c", "--components", help="list the diffents ports associated to component",
    #                     action="store_true")
    parser.add_argument("-a", "--all",
                        help="show all ports configured for an interface instead of showing only those with more than one interface per port",
                        action="store_true")
    parser.add_argument("-l", "--lower",
                        help="show all ports with numbers lower than 10000",
                        action="store_true")
    parser.add_argument("-i", "--interface",
                        help="List only interfaces that contains this string",
                        type=str)
    parser.add_argument('action', choices=('ports', 'interfaces'), help="Show the interfaces by name or by port")
    parser.add_argument('path', nargs='?',
                        help="path to look for components config files recursively (default=\"~/robocomp/\")")
    args = parser.parse_args()

    if args.interface is not None and args.action == "ports":
        print(
                BColors.WARNING + "[!] Wrong parameters combination: Filtering an interface by name (-i) while listing ports is not available." + BColors.ENDC)
        parser.print_help()
        sys.exit()

    rcportchecker = RCPortChecker(args.verbose, args.path)

    if args.action == "ports":
        if args.port is not None:
            rcportchecker.print_port_info(args.port, args.all, args.lower, args.interface)
        else:
            rcportchecker.print_port_listing(args.all, args.lower, args.interface)
    elif args.action == "interfaces":
        rcportchecker.print_interface_listing(args.all, args.lower, args.interface)


if __name__ == '__main__':
    main()
