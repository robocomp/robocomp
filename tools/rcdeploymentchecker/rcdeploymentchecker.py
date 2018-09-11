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
import pprint
import re
import sys
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(CURRENT_DIR,"../rcportchecker"))
from rcportchecker import RCPortChecker

print(RCPortChecker)
try:
	import xml.etree.cElementTree as ET
except ImportError:
	import xml.etree.ElementTree as ET


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


def main():

	parser = MyParser(description='Application to look for existing configured interfaces ports on components')
	# parser.add_argument("-v", "--verbose", help="increase output verbosity",
	# 					action="store_true")
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
	parser.add_argument('path', nargs='?',
						help="path to look for components config files recursively (default=\"~/robocomp/\")")
	args = parser.parse_args()

	interfaces_to_check = {}
	remote_interfaces = {}
	suspicious_interfaces = {}
	local = False
	print("Parsing %s"%args.path)
	tree = ET.ElementTree(file=args.path)
	root = tree.getroot()
	if root.tag != "rcmanager":
		print("[!] It's not valid deployment file")
		sys.exit()
	for node in tree.iterfind('node'):
		endpoint_string =  node.attrib["endpoint"]
		# print endpoint_string
		ip = re.findall(r'(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})', endpoint_string)
		if "localhost" in endpoint_string.lower() or len(ip)==0 or "127.0.0.1" in ip[0] :
			local = True
		for upcommand in node.iterfind('upCommand'):
			suspicious = False
			upcommand_string = upcommand.attrib["command"]
			upcommand_parts = upcommand_string.split(" ")
			config_file_path = upcommand_parts[-1]
			if "--Ice.Config".lower() in config_file_path.lower() and "=" in config_file_path:
				config_file_path = config_file_path.split("=")[-1].strip()
			if len(config_file_path)>0:
				if config_file_path[0] != "/":
					print("Possible relative config file path? %s for endpoint %s" % (upcommand_string, endpoint_string))
					suspicious = True
			else:
				print("No config file path? %s for endpoint %s" % (upcommand_string, endpoint_string))
				suspicious = True
			if "/" in config_file_path:
				filename = config_file_path.split("/")[-1]
				name, file_extension = os.path.splitext(filename)
			else:
				print("config file path without \"/\"? %s for endpoint %s" % (upcommand_string, endpoint_string))
				suspicious = True
			if suspicious is True:
				if endpoint_string in suspicious_interfaces:
					suspicious_interfaces[endpoint_string].append(config_file_path)
				else:
					suspicious_interfaces[endpoint_string] = [config_file_path]
			elif (("config" in name and file_extension == '') or (".conf" in file_extension)) and "etc" in config_file_path:
				if local:
					if endpoint_string in interfaces_to_check:
						interfaces_to_check[endpoint_string].append(config_file_path)
					else:
						interfaces_to_check[endpoint_string] = [config_file_path]
				else:
					if endpoint_string in remote_interfaces:
						remote_interfaces[endpoint_string].append(config_file_path)
					else:
						remote_interfaces[endpoint_string] = [config_file_path]

	rcpc = RCPortChecker()
	# print("---------------")
	# print("Remote to ckeck in remote machines")
	# pprint.pprint(remote_interfaces)
	# print("---------------")
	# print("Suspicious")
	# pprint.pprint(suspicious_interfaces)
	# print("---------------")
	# print("To check")
	# pprint.pprint(interfaces_to_check)
	for endpoint, paths in interfaces_to_check.items():
		endpoint_name = endpoint.split(":")[0]
		endpoint_port = int(re.findall(r'-p\s*(\d+)', endpoint)[0])
		for path in paths:
			if os.path.exists(path) and os.path.isfile(path):
				rcpc.clear()
				rcpc.parse_config_file(path)
				config_port, config_paths = rcpc.search_interface_port(endpoint_name)
				if config_port is None or endpoint_port != config_port:
					print("WRONG PORT %s vs %s for endpoint %s in config file %s"%(str(endpoint_port), str(config_port), endpoint, path))


		# print elem.tag, elem.attrib

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