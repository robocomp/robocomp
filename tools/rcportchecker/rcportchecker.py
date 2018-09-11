#!/usr/bin/env python

import getopt
import argparse
import os
import re
import sys



class MyParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write('error: %s\n' % message)
        self.print_help()
        sys.exit(2)

def default_dir(debug=False):
    default_robocomp_path = os.path.join(os.path.expanduser("~"), "robocomp")
    if debug:
        print('Trying default directory (%s)' % (default_robocomp_path))
    if os.path.exists(default_robocomp_path):
        if debug:
            print("Default Robocomp (%s) exists " % (default_robocomp_path))
        if os.path.isdir(default_robocomp_path):
            if debug:
                print("Default Robocomp directory (%s) is a directory " % (default_robocomp_path))
            if not os.listdir(default_robocomp_path):
                print("Default Robocomp directory (%s) exists but it's empty. Exiting!" % (default_robocomp_path))
                sys.exit()
            else:
                return default_robocomp_path
        else:
            print("Default Robocomp directory (%s) exists but it's not a directory. Exiting!" % (default_robocomp_path))
            sys.exit()
    else:
        print("Default Robocomp directory (%s) doesn't exists. Exiting!" % (default_robocomp_path))
        sys.exit()

def print_interfaces_and_paths(interfaces, arg_interface):
    for interface, paths in interfaces.items():
        if arg_interface is not None and arg_interface not in interface.lower():
            continue
        print
        "\t%s" % (interface)
        for path in paths:
            print
            "\t\t%s" % (path)


def main(name, argv):
    # help_string = "%s [-h] [-p] [-a]"%(os.path.basename(name))
    # try:
    #     opts, args = getopt.getopt(argv,"hpav",["help","ports","all", "verbose"])
    # except getopt.GetoptError:
    #     print help_string
    #     sys.exit(2)


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
    parser.add_argument('path', nargs='?', help="path to look for components config files recursively (default=\"~/robocomp/\")")
    args = parser.parse_args()
    if args.path:
        robocomp_path = args.path
    else:
        robocomp_path = default_dir(args.verbose)
    interfaces_ports = {}
    ports_for_interfaces = {}
    for root, dirs, files in os.walk(robocomp_path, topdown=False):
        for name in files:
            file_name, file_extension = os.path.splitext(name)
            if ("config" in name and file_extension == '') or (".conf" in file_extension) and "etc" in root:
                fullpath = os.path.join(root, name)
                with open(fullpath, 'r') as fin:
                    for line in fin:
                        if ".Endpoints" in line and "cog.outl" not in line:
                            if args.verbose:
                                print "Looking into file %s" % (fullpath)
                            extracted = re.findall(r'(.*)\.Endpoints\s*=.*-p\s?(\d+)', line)
                            if extracted:
                                if args.verbose:
                                    print "\tfound endpoint %s" % (line)
                                (interface_name, port) = extracted[0]
                                port = int(port)
                                if interface_name in interfaces_ports:
                                    if port not in interfaces_ports[interface_name].keys():
                                        interfaces_ports[interface_name][port] = [fullpath]
                                    else:
                                        interfaces_ports[interface_name][port].append(fullpath)
                                else:
                                    interfaces_ports[interface_name] = {}
                                    interfaces_ports[interface_name][port] = [fullpath]

                                if port in ports_for_interfaces:
                                    if interface_name not in ports_for_interfaces[port]:
                                        ports_for_interfaces[port][interface_name]=[fullpath]
                                    else:
                                        ports_for_interfaces[port][interface_name].append(fullpath)
                                else:
                                    ports_for_interfaces[port] ={}
                                    ports_for_interfaces[port][interface_name]=[fullpath]
                            else:
                                print("Interface without port? %s" % (line))

    # pprint.pprint(interfaces_ports)
    # pprint.pprint(ports_for_interfaces)
    if args.verbose:
        print "\n---\n"
    if args.action == "ports":
        if args.port is not None:
            if args.port in ports_for_interfaces:
                interfaces = ports_for_interfaces[args.port]
                to_show = True
                if not args.all and len(interfaces) < 2:
                    to_show = False
                if args.lower and port > 10000:
                    to_show = False
                if to_show:
                    print("In port %d\t" % (port))
                    print_interfaces_and_paths(interfaces, args.interface)
            else:
                print("Port %d not found"%(args.port))
        else:
            for port, interfaces in sorted(ports_for_interfaces.items()):
                to_show = True
                if not args.all and len(interfaces) < 2:
                    to_show = False
                if args.lower and port > 10000:
                    to_show = False
                if to_show:
                    print("In port %d\t" % (port))
                    print_interfaces_and_paths(interfaces, args.interface)
    elif args.action == "interfaces":
        for interface, ports in sorted(interfaces_ports.items()):
            if args.interface is not None and args.interface.lower() not in interface.lower():
                continue
            to_show = True
            if not args.all and len(ports) < 2:
                to_show = False
            if to_show:
                print("%s" % (interface))
                for port, paths in ports.items():
                    if args.lower and port > 10000:
                        to_show = False
                    else:
                        to_show = True
                    if to_show:
                        print "\t%d" % (port)
                        for path in paths:
                            print "\t\t%s" % (path)





if __name__ == '__main__':
    main(sys.argv[0], sys.argv[1:])
