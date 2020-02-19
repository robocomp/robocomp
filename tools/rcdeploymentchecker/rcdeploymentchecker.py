#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
#  ------------------------
#  ----- rcdeploymentchecker -----
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
import argparse
import os
import re
import sys

# DETECT THE ROBOCOMP INSTALLATION TO IMPORT RCPORTCHECKER CLASS
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    default_robocomp_path = '/opt/robocomp'
    print('ROBOCOMP environment variable not set! Trying default directory (%s)' % (default_robocomp_path))
    if os.path.exists(default_robocomp_path) and os.path.isdir(default_robocomp_path):
        if not os.listdir(default_robocomp_path):
            print("Default Robocomp directory (%s) exists but it's empty. Exiting!" % (default_robocomp_path))
            sys.exit()
        else:
            ROBOCOMP = default_robocomp_path
    else:
        print("Default Robocomp directory (%s) doesn't exists. Exiting!" % (default_robocomp_path))
        sys.exit()
sys.path.append(os.path.join(ROBOCOMP, "python"))
from rcportchecker import RCPortChecker, BColors

try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET


# PRINTING WITH COLORS
class BColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class RExp:
    HOST = '(?:localhost|(?:\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}))'
    PATH = '[\\\/]?(?:[^\\^\\/^\\s]+[\\\/]?)+'
    COMMAND = '(?:(?:python\\s+)?' + PATH + "|rcnode)"


class MyParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write((BColors.FAIL + 'error: %s\n' + BColors.ENDC) % message)
        self.print_help()
        sys.exit(2)


class RCDeploymentChecker:
    def __init__(self, debug):
        self.debug = debug
        self.interfaces_to_check = {}
        self.remote_interfaces = {}
        self.suspicious_interfaces = {}
        self.rcpc = RCPortChecker()

    def parse_deployment_file(self, path):
        if self.debug:
            print("[+] Parsing %s" % path)
        # Read the xml file
        tree = ET.ElementTree(file=path)
        root = tree.getroot()

        # Check it's a rcmanager file
        if root.tag != "rcmanager":
            print("[!] It's not valid deployment file")
            sys.exit()

        # Look for nodes of xml file containing endpoints
        for node in tree.iterfind('node'):
            endpoint_string = node.attrib["endpoint"]
            # look for ip on the endpoint string
            ip = re.findall(r'(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', endpoint_string)
            if "localhost" in endpoint_string.lower() or len(ip) == 0 or "127.0.0.1" in ip[0]:
                if self.debug:
                    print("[+] Found local node: %s" % endpoint_string)
                local = True
            else:
                if self.debug:
                    print("[+] Found REMOTE node: %s" % endpoint_string)
                local = False

            # Look for the upcommand strings
            for upcommand in node.iterfind('upCommand'):
                suspicious = False
                upcommand_string = upcommand.attrib["command"]
                if self.debug:
                    print("\t[+] Upcommand: %s" % upcommand_string)
                upcommand_struct = self.parse_upcommand_line(upcommand_string)
                if upcommand_struct is not None:
                    if "component_config" in upcommand_struct:
                        config_file_path = upcommand_struct["component_config"]
                        if config_file_path is not None and len(config_file_path) > 0:
                            if config_file_path[0] != "/":
                                if self.debug:
                                    print(
                                                "\t" + BColors.WARNING + "[?]" + BColors.ENDC + " Possible relative config file path? %s for endpoint %s" % (
                                            upcommand_string, endpoint_string))
                                suspicious = True
                            if local:
                                if "/" in config_file_path:
                                    filename = config_file_path.split("/")[-1]
                                    name, file_extension = os.path.splitext(filename)
                                    if name is not None and file_extension is not None:
                                        if (("config" in name and file_extension == '') or (
                                                ".conf" in file_extension)) and "etc" in config_file_path:
                                            self.add_to_checkeables(endpoint_string, config_file_path)
                                        else:
                                            pass
                                else:
                                    if self.debug:
                                        print(
                                                    "\t" + BColors.WARNING + "[?]" + BColors.ENDC + " Config file path without \"/\"? upcommand=\"%s\" for endpoint %s" % (
                                                upcommand_string, endpoint_string))
                                    suspicious = True
                            else:
                                self.add_to_remotes(endpoint_string, config_file_path)

                            if suspicious is True:
                                if self.debug:
                                    print(
                                                "\t" + BColors.WARNING + "[?]" + BColors.ENDC + " Weird config file name or not found at all %s for endpoint %s" % (
                                            upcommand_string, endpoint_string))
                        else:
                            if self.debug:
                                print(
                                            "\t" + BColors.WARNING + "[?]" + BColors.ENDC + " No config file path? upcommand=\"%s\" for endpoint %s" % (
                                        upcommand_string, endpoint_string))
                            suspicious = True
                    else:
                        if self.debug:
                            print(
                                        "\t" + BColors.WARNING + "[?]" + BColors.ENDC + " No config file? \"%s\" upcommand for endpoint %s" % (
                                    upcommand_string, endpoint_string))
                        suspicious = True

                    if suspicious:
                        self.add_to_suspicious(endpoint_string, config_file_path)

                    name = None
                    file_extension = None

                else:
                    self.add_to_suspicious(endpoint_string, upcommand_string)
        if self.debug:
            print("\n--------\n")

    def add_to_remotes(self, endpoint_string, config_file_path):
        if endpoint_string in self.remote_interfaces:
            self.remote_interfaces[endpoint_string].append(config_file_path)
        else:
            self.remote_interfaces[endpoint_string] = [config_file_path]

    # Parse the upcommand line and return it as structured
    def parse_upcommand_line(self, upcommand_string):
        is_full_path = False
        config_path = None
        upcommand = {}
        if "rcremote" in upcommand_string.lower():
            result = re.match(
                r"rcremote\s+(" + RExp.HOST + ")\s+(\w+)\s(" + RExp.PATH + ")\s+(" + RExp.COMMAND + ")(?:\s+(.*))?",
                upcommand_string)
            if result is not None:
                upcommand["rcremote"] = True
                upcommand["machine"] = result.group(1)
                upcommand["component_name"] = result.group(2)
                upcommand["component_path"] = result.group(3)
                upcommand["component_command"] = result.group(4)
                upcommand["component_config"] = result.group(5)
                upcommand["component_config"] = self.extract_upcommand_config_file_full_path(upcommand)
                upcommand["component_path"] = self.extract_upcommand_component_full_path(upcommand)
                result.group(3)
                return upcommand
            else:
                if self.debug:
                    print("Bad rcremote command")
                return None
        elif upcommand_string.strip() == "":
            return None
        elif re.match(RExp.PATH, upcommand_string):
            command_parts = upcommand_string.split(" ")
            upcommand["rcremote"] = False
            upcommand["machine"] = "localhost"
            upcommand["component_name"] = command_parts[0].split("/")[-1]
            upcommand["component_path"] = command_parts[0]
            upcommand["component_command"] = command_parts[0]
            if len(command_parts) > 1:
                upcommand["component_config"] = command_parts[1]
                upcommand["component_config"] = self.extract_upcommand_config_file_full_path(upcommand)
            upcommand["component_path"] = self.extract_upcommand_component_full_path(upcommand)
            return upcommand
        else:
            print("WTF!")
            sys.exit()
        # if "/" in config_file_path:
        # 	filename = config_file_path.split("/")[-1]
        # 	name, file_extension = os.path.splitext(filename)
        # else:
        # 	print("\t[?] Config file path without \"/\"? upcommand=\"%s\" for endpoint %s" % (
        # 		upcommand_string, endpoint_string))

    # def process_config_string(self, config_string):
    # 	result = ""
    # 	if config_string != None:
    # 		config_path_string = config_string
    # 		if "--Ice.Config".lower() in config_path_string.lower():
    # 			if "=" in config_path_string:
    # 				result = config_path_string.split("=")[-1]
    # 			else:
    # 				print("[!] Bad formatted string in upcommand. No '=' after --Ice.Config: %s" % (config_path_string))
    # 		result = self.extract_upcommand_config_file_full_path(upcommand)
    # 	return result

    def extract_upcommand_config_file_full_path(self, upcommand):
        full_path = upcommand["component_config"]
        if full_path is None:
            full_path = ''
        if len(full_path) > 0:
            if "--Ice.Config".lower() in full_path.lower():
                if "=" in full_path:
                    full_path = full_path.split("=")[-1]
            if "etc" in full_path:
                if len(upcommand["component_path"]) > 0 and upcommand["component_path"][0] == '/':
                    full_path = os.path.join(upcommand["component_path"], full_path)
        if len(upcommand["component_path"]) > 0 and upcommand["component_path"][0] == '/':
            full_path = os.path.join(upcommand["component_path"], full_path)
        return full_path

    def extract_upcommand_component_full_path(self, upcommand):
        full_path = upcommand["component_path"]
        if len(full_path) == 0 or full_path[0] != "/":
            if "component_config" in upcommand:
                config_path = upcommand["component_config"]
                if len(config_path) > 0 and config_path[0] == '/' and '/etc/' in config_path:
                    print(re.sub(r'.*/etc/.*', '', config_path))
                    full_path = re.sub(r'/etc/.*', '', config_path)
                else:
                    full_path = ''
            else:
                full_path = ''
        return full_path

    def print_local_interfaces_check(self):
        for endpoint, paths in self.interfaces_to_check.items():
            endpoint_name = endpoint.split(":")[0]
            endpoint_port = int(re.findall(r'-p\s*(\d+)', endpoint)[0])
            for path in paths:
                result, data = self.check_endpoint_in_config_file(endpoint, path)
                if result == 2:
                    matched_port_interface, _ = data
                    print((BColors.WARNING + "[?]" + BColors.ENDC +
                           " Need check: Found an interface configured on port " +
                           BColors.OKGREEN + "%d." + BColors.ENDC + " Is " +
                           BColors.OKBLUE + "%s" + BColors.ENDC + " the interface for the endpoint " +
                           BColors.OKBLUE + "%s" + BColors.ENDC + " (%s - %s)") %
                          (endpoint_port,
                           matched_port_interface,
                           endpoint,
                           matched_port_interface,
                           endpoint_name))
                elif result == 0:
                    config_port, _ = data
                    print(
                                BColors.FAIL + "[!]" + BColors.ENDC + " WRONG PORT %s vs %s for endpoint %s in config file %s" % (
                            str(endpoint_port), str(config_port), endpoint, path))
                elif result == 1:
                    if self.debug:
                        print("[+] Matching ports for endpoint %s" % endpoint)
                elif result == -1:
                    print((BColors.FAIL + "[!]" + BColors.ENDC + " Not found: Expected path %s for endpoint %s") % (
                    path, endpoint))

    def check_endpoint_in_config_file(self, endpoint, path):
        endpoint_name = endpoint.split(":")[0]
        endpoint_port = int(re.findall(r'-p\s*(\d+)', endpoint)[0])
        if os.path.exists(path) and os.path.isfile(path):
            self.rcpc.clear()
            self.rcpc.parse_config_file(path)
            config_port, config_paths = self.rcpc.search_interface_port_by_name(endpoint_name)
            # Matched
            if config_port is not None and endpoint_port == config_port:
                return 1, [config_port, config_paths]
            else:
                # Not matche
                matched_port_interface, config_paths_2 = self.rcpc.search_interface_by_port(endpoint_port)
                # Look interface in same port
                if matched_port_interface is not None:
                    return 2, [matched_port_interface, config_paths_2]
                else:
                    return 0, [config_port, config_paths]
        else:
            return -1, []

    def add_to_suspicious(self, endpoint_string, config_file_path):
        if endpoint_string in self.suspicious_interfaces:
            self.suspicious_interfaces[endpoint_string].append(config_file_path)
        else:
            self.suspicious_interfaces[endpoint_string] = [config_file_path]

    def add_to_checkeables(self, endpoint_string, config_file_path):
        if endpoint_string in self.interfaces_to_check:
            self.interfaces_to_check[endpoint_string].append(config_file_path)
        else:
            self.interfaces_to_check[endpoint_string] = [config_file_path]


def main():
    parser = MyParser(description='Application to check and existing deployment xml file for ports and endpoints')
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                        action="store_true")
    # parser.add_argument("-p", "--port", help="List only the selected port information",
    # 					type=int)
    # # parser.add_argument("-c", "--components", help="list the diffents ports associated to component",
    # #                     action="store_true")
    # parser.add_argument("-a", "--all",
    # 					help="show all ports configured for an interface instead of showing only those with more than one interface per port",
    # 					action="store_true")
    # parser.add_argument("-l", "--lower",
    # 					help="show all ports with numbers lower than 10000",
    # 					action="store_true")
    # parser.add_argument("-i", "--interface",
    # 					help="List only interfaces that contains this string",
    # 					type=str)
    # parser.add_argument('action', choices=('ports', 'interfaces'), help="Show the interfaces by name or by port")
    parser.add_argument('path',
                        help="path to look for deployment file (xml of rcmanager tool)")
    args = parser.parse_args()

    rcdeplymentchecker = RCDeploymentChecker(args.verbose)
    rcdeplymentchecker.parse_deployment_file(args.path)
    rcdeplymentchecker.print_local_interfaces_check()



# print("---------------")
# print("Remote to ckeck in remote machines")
# pprint.pprint(remote_interfaces)
# print("---------------")
# print("Suspicious")
# pprint.pprint(suspicious_interfaces)
# print("---------------")
# print("To check")
# pprint.pprint(interfaces_to_check)


# if args.interface is not None and args.action == "ports":
# 	print(
# 		BColors.WARNING + "[!] Wrong parameters combination: Filtering an interface by name (-i) while listing ports is not available." + BColors.ENDC)
# 	parser.print_help()
# 	sys.exit()
#
# rcportchecker = RCPortChecker(args.verbose, args.path)
#
# if args.action == "ports":
# 	if args.port is not None:
# 		rcportchecker.print_port_info(args.port, args.all, args.lower, args.interface)
# 	else:
# 		rcportchecker.print_port_listing(args.all, args.lower, args.interface)
# elif args.action == "interfaces":
# 	rcportchecker.print_interface_listing(args.all, args.lower, args.interface)


if __name__ == '__main__':
    main()
