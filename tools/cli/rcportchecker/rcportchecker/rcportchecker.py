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
from pathlib import Path
from typing import Optional

import typer
import os
import re
import sys
from collections import defaultdict


from rich.console import Console
console = Console()
app = typer.Typer()
MIN_ROBOCOMP_PORT = 10000

class BColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# class MyParser(argparse.ArgumentParser):
#     def error(self, message):
#         sys.stderr.write((BColors.FAIL + 'error: %s\n' + BColors.ENDC) % message)
#         self.print_help()
#         sys.exit(2)


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

    def search_interfaces_by_port(self, port):
        if port in self.ports_for_interfaces.keys():
            interfaces = self.ports_for_interfaces[port]
            return interfaces
        else:
            return None

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
                if line.lstrip().startswith('#'):
                    continue
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

    def fix_not_matching_ports(self):
        to_pop = []
        to_fix = []
        ports_assigned = {"": 0}
        for port, interfaces in sorted(self.ports_for_interfaces.items()):
            if len(interfaces) > 1 or port == 0:
                console.print(f"Collision of {len(interfaces)} interfaces in port {port} {interfaces.keys()}", style="yellow")
                for interface_name in interfaces:
                    fixed = False
                    suggested_port = self.suggest_port_for_interface(interface_name, ports_assigned)
                    for file_to_fix in interfaces[interface_name]:
                        if port not in self.ports_for_interfaces[suggested_port] or file_to_fix not in self.ports_for_interfaces[suggested_port][interface_name]:
                            to_fix.append((port, suggested_port, interface_name, file_to_fix))
                            fixed = True
                    if port != suggested_port and fixed:
                        to_pop.append((port, interface_name))
                        ports_assigned[interface_name] = suggested_port
        for old_port, new_port, interface_name, file_to_fix in to_fix:
            self.replace_port_in_file(file_to_fix, interface_name, old_port, new_port)


    def replace_port_in_file(self, fullpath, interface_name, new_port, old_port = None):
        console.log(f"In file {fullpath}")
        with open(fullpath, 'r') as fin:
            for line in fin:
                if line.lstrip().startswith('#'):
                    continue
                if ".Endpoints" in line and "cog.outl" not in line:
                    if self.debug:
                        print("Looking into file %s" % (fullpath))
                    if interface_name in line:
                        if old_port is not None:
                            if (old_port) in line:
                                console.log(f"would replace\t {line.rstrip()}")
                                new_line = line.replace(str(old_port), str(new_port))
                                console.log(f"with\t\t {new_line.rstrip()}")
                            else:
                                console.log(f"would replace\t {line.rstrip()}")
                                new_line = re.sub(f'({interface_name}.* -p\s?)\d+ ',  f'\1{new_port}',    line)
                                console.log(f"with\t\t {new_line.rstrip()}")

    def suggest_port_for_interface(self, interface_name, already_assigned):
        if interface_name in already_assigned:
            console.log(f"Previously assigned port {already_assigned[interface_name]} suggested for {interface_name}.")
            return already_assigned[interface_name]
        ports_for_interface = self.search_interface_ports_by_name(interface_name)
        # Check how many different ports are used for the interface and how many times
        times_interface_in_port = {key: len(value) for key, value in ports_for_interface.items()}
        # For each used port ordered by the number of times it's used (best candidate)
        for suggested_port, times_repeated in sorted(times_interface_in_port.items(), key=lambda x: x[1], reverse=True):
            if suggested_port in already_assigned or suggested_port < MIN_ROBOCOMP_PORT:
                continue
            # It's checked if other interface is using that port
            interfaces_in_port = self.search_interfaces_by_port(suggested_port)
            # Exclude current interface
            interfaces_in_port = {x: interfaces_in_port[x] for x in interfaces_in_port if x != interface_name}
            # if not used by any other, suggest
            if interfaces_in_port is None or len(interfaces_in_port) == 0:
                console.log(f"Not collisional port {suggested_port} suggested for {interface_name}.")
                return suggested_port
            # If it's used by other but less times than for this interface, suggest
            elif times_repeated > max((len(v), k) for k, v in interfaces_in_port.items())[0]:
                console.log(f"Because it's repeated {times_repeated} time, port {suggested_port} suggested for {interface_name}.")
                return suggested_port
            else:
                times_repeated_2, interface = max((len(v), k) for k, v in interfaces_in_port.items())
                console.log(f"REJECTED: Port {suggested_port} not suggested because {interface} use it {times_repeated_2} times")
        # get sequential port
        # get interfaces in that port
        # if no collision suggest
        return self.next_secuential_port(interface_name, already_assigned.values())

    def next_secuential_port(self, interface_name, already_assigned_ports):
        for suggested_port in sorted(list(self.ports_for_interfaces.keys())+list(already_assigned_ports)):
            if suggested_port >= MIN_ROBOCOMP_PORT and \
                    suggested_port + 1 not in already_assigned_ports:
                    # suggested_port + 1 not in self.ports_for_interfaces and\
                console.log(f"New sequential port {suggested_port+1} suggested for {interface_name}.")
                return suggested_port + 1
        return -1

    def other(self, to_classify = None, filled_ports = None):
        # console.log(sorted(self.ports_for_interfaces))
        # console.log(self.interfaces_ports)
        if filled_ports is None:
            new_ports_list = {}
        else:
            new_ports_list = filled_ports
        unclassified = []
        if to_classify is None:
            to_classify = self.interfaces_ports.items()
        for interface,_ in to_classify:
            ports = self.interfaces_ports[interface]
            sorted_ports = sorted(ports.items(), key=lambda x: len(x[1]), reverse=True)
            assigned = False
            occupied = []
            for port, files in sorted_ports:
                if port < MIN_ROBOCOMP_PORT:
                    continue
                if port not in new_ports_list:
                    new_ports_list[port] = (interface, len(files))
                    assigned = True
                    break
                elif len(files) > new_ports_list[port][1]:
                    unclassified.append(new_ports_list[port])
                    new_ports_list[port] = (interface, len(files))
                    assigned = True
                    break
                else:
                    occupied.append((port, new_ports_list[port]))
            if not assigned:
                console.log(f"Need new port {interface} {list(ports.keys())}")
                if occupied:
                    console.log(f"\tOcuppied by {occupied}")
                suggested_port = self.next_secuential_port(interface, new_ports_list.keys())
                new_ports_list[suggested_port] = (interface, len(files))


        if len(unclassified) > 0:
            return self.other(unclassified,new_ports_list)
        else:
            console.log(sorted(new_ports_list.items()))
            all_interface_names = [x[0] for x in new_ports_list.values()]
            for interface in self.interfaces_ports:
                if interface not in all_interface_names:
                    console.log(f"{interface} is in initial but not in last", style='red')
            inverted_dict = {v[0]: k for k, v in new_ports_list.items()}
            return inverted_dict



@app.command(name="ports")
def ports(port: Optional[int] = None,
          path: Optional[Path] = None,
          lower: bool = typer.Option(False, "--lower", "-l", help="show all ports with numbers lower than 10000"),
          verbose: bool = typer.Option(False, "--verbose", "-v", help="increase output verbosity"),
          show_all: bool = typer.Option(False, "--all", "-a", help="show all ports configured for an interface instead of showing only those with more than one interface per port"),
          interface: str = ""
          ):
    rcportchecker = RCPortChecker(verbose, path)
    if port is not None:
        rcportchecker.print_port_info(port, show_all, lower, interface)
    else:
        rcportchecker.print_port_listing(show_all, lower, interface)

@app.command(name="fix")
def fix():
    rcportchecker = RCPortChecker(verbose, path)
    rcportchecker.other()

@app.command(name="interfaces")
def interfaces(
        path: Optional[Path] = None,
        lower: bool = typer.Option(False, "--lower", "-l", help="show all ports with numbers lower than 10000"),
        verbose: bool = typer.Option(False, "--verbose", "-v", help="increase output verbosity"),
        show_all: bool = typer.Option(False, "--all", "-a", help="show all ports configured for an interface instead of showing only those with more than one interface per port"),
        interface: str = ""
        ):
    rcportchecker = RCPortChecker(verbose, path)
    rcportchecker.print_interface_listing(show_all, lower, interface)

# def main():
#     parser = MyParser(description='Application to look for existing configured interfaces ports on components')
#     parser.add_argument("-v", "--verbose", help="increase output verbosity",
#                         action="store_true")
#     parser.add_argument("-p", "--port", help="List only the selected port information",
#                         type=int)
#     # parser.add_argument("-c", "--components", help="list the diffents ports associated to component",
#     #                     action="store_true")
#     parser.add_argument("-a", "--all",
#                         help="show all ports configured for an interface instead of showing only those with more than one interface per port",
#                         action="store_true")
#     parser.add_argument("-l", "--lower",
#                         help="show all ports with numbers lower than 10000",
#                         action="store_true")
#     parser.add_argument("-i", "--interface",
#                         help="List only interfaces that contains this string",
#                         type=str)
#     parser.add_argument('action', choices=('ports', 'interfaces', 'fix'), help="Show the interfaces by name or by port")
#     parser.add_argument('path', nargs='?',
#                         help="path to look for components config files recursively (default=\"~/robocomp/\")")
#     args = parser.parse_args()
#
#     rcportchecker = RCPortChecker(args.verbose, args.path)
#
#     if args.action == "fix":
#         rcportchecker.other()
#     else:
#         if args.interface is not None and args.action == "ports":
#             print(
#                 BColors.WARNING + "[!] Wrong parameters combination: Filtering an interface by name (-i) while listing ports is not available." + BColors.ENDC)
#             parser.print_help()
#             sys.exit()
#         if args.action == "ports":
#             if args.port is not None:
#                 rcportchecker.print_port_info(args.port, args.all, args.lower, args.interface)
#             else:
#                 rcportchecker.print_port_listing(args.all, args.lower, args.interface)
#         elif args.action == "interfaces":
#             rcportchecker.print_interface_listing(args.all, args.lower, args.interface)




if __name__ == '__main__':
    app()
