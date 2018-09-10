#!/usr/bin/env python

import getopt
import argparse
import os
import re
import sys


# default_robocomp_path = '~/robocomp/'
# print('Trying default directory (%s)'%(default_robocomp_path))
# if os.path.exists(default_robocomp_path):
#     print '%s path exists'%(default_robocomp_path)
#     if os.path.isdir(default_robocomp_path):
#         print '%s is a directory'%(default_robocomp_path)
#         if not os.listdir(default_robocomp_path):
#             print("Default Robocomp directory (%s) exists but it's empty. Exiting!"%(default_robocomp_path))
#             sys.exit()
# else:
#     print("Default Robocomp directory (%s) doesn't exists. Exiting!"%(default_robocomp_path))
#     sys.exit()

def default_dir():
    default_robocomp_path = os.path.join(os.path.expanduser("~"), "robocomp")
    print('Trying default directory (%s)' % (default_robocomp_path))
    if os.path.exists(default_robocomp_path):
        print("Default Robocomp (%s) exists " % (default_robocomp_path))
        if os.path.isdir(default_robocomp_path):
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


def main(name, argv):
    # help_string = "%s [-h] [-p] [-a]"%(os.path.basename(name))
    # try:
    #     opts, args = getopt.getopt(argv,"hpav",["help","ports","all", "verbose"])
    # except getopt.GetoptError:
    #     print help_string
    #     sys.exit(2)

    parser = argparse.ArgumentParser(description='Application to look for existing configured ports on components')
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                        action="store_true")
    parser.add_argument("-p", "--port", help="List only the selected port information",
                        type=int)
    # parser.add_argument("-c", "--components", help="list the diffents ports associated to component",
    #                     action="store_true")
    parser.add_argument("-a", "--all",
                        help="show all ports configured in a component instead of showing only those with more than one component per port",
                        action="store_true")
    parser.add_argument("-l", "--lower",
                        help="show all ports with numbers lower than 10000",
                        action="store_true")
    parser.add_argument('action', choices=('ports', 'comps'), help="Show the components by name or by port")
    parser.add_argument('path', nargs='?', help="path to look for components config files recursively (default=\"~/robocomp/\")")
    try:
        args = parser.parse_args()
    except:
        parser.print_help()
        sys.exit(0)

    if args.path:
        robocomp_path = args.path
    else:
        robocomp_path = default_dir()
    components_ports = {}
    ports_for_components = {}
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
                                (comp_name, port) = extracted[0]
                                port = int(port)
                                if comp_name in components_ports:
                                    if port not in components_ports[comp_name].keys():
                                        components_ports[comp_name][port] = [fullpath]
                                    else:
                                        components_ports[comp_name][port].append(fullpath)
                                else:
                                    components_ports[comp_name] = {}
                                    components_ports[comp_name][port] = [fullpath]

                                if port in ports_for_components:
                                    if comp_name not in ports_for_components[port]:
                                        ports_for_components[port][comp_name]=[fullpath]
                                    else:
                                        ports_for_components[port][comp_name].append(fullpath)
                                else:
                                    ports_for_components[port] ={}
                                    ports_for_components[port][comp_name]=[fullpath]
                            else:
                                print("Component without port? %s" % (line))

    # pprint.pprint(components_ports)
    # pprint.pprint(ports_for_components)
    if args.verbose:
        print "\n---\n"
    if args.action == "ports":
        if args.port is not None:
            if args.port in ports_for_components:
                components = ports_for_components[args.port]
                to_show = True
                if not args.all and len(components) < 2:
                    to_show = False
                if args.lower and port > 10000:
                    to_show = False
                if to_show:
                    print "In port %d\t" % (args.port)
                    for component, paths in components.items():
                        print "\t%s" % (component)
                        for path in paths:
                            print "\t\t%s" % (path)
            else:
                print "Port %d not found"%(args.port)
        else:
            for port, components in sorted(ports_for_components.items()):
                to_show = True
                if not args.all and len(components) < 2:
                    to_show = False
                if args.lower and port > 10000:
                    to_show = False
                if to_show:
                    print "In port %d\t" % (port)
                    for component, paths  in components.items():
                        print "\t%s"%(component)
                        for path in paths:
                            print "\t\t%s" % (path)
    elif args.action == "comps":
        for component, ports in sorted(components_ports.items()):
            to_show = True
            if not args.all and len(ports) < 2:
                to_show = False
            if to_show:
                print "%s" % (component)
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
