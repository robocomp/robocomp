#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
#    Copyright (C) 2021 by YOUR NAME HERE
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

# \mainpage RoboComp::testcomp
#
# \section intro_sec Introduction
#
# Some information about the component...
#
# \section interface_sec Interface
#
# Descroption of the interface provided...
#
# \section install_sec Installation
#
# \subsection install1_ssec Software depencences
# Software dependences....
#
# \subsection install2_ssec Compile and install
# How to compile/install the component...
#
# \section guide_sec User guide
#
# \subsection config_ssec Configuration file
#
# <p>
# The configuration file...
# </p>
#
# \subsection execution_ssec Execution
#
# Just: "${PATH_TO_BINARY}/testcomp --Ice.Config=${PATH_TO_CONFIG_FILE}"
#
# \subsection running_ssec Once running
#
#
#

import argparse
# Ctrl+c handling
import signal

from rich.console import Console
console = Console()

from PySide2 import QtCore
from PySide2 import QtWidgets
import interfaces
from specificworker import *

#SIGNALS handler
def sigint_handler(*args):
    QtCore.QCoreApplication.quit()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('iceconfigfile', nargs='?', type=str, default='etc/config')
    parser.add_argument('--startup-check', action='store_true')

    args = parser.parse_args()
    interface_manager = interfaces.InterfaceManager(args.iceconfigfile)

    if interface_manager.status == 0:
        worker = SpecificWorker(interface_manager.get_proxies_map(), args.startup_check)
        worker.setParams(interface_manager.parameters)
    else:
        print("Error getting required connections, check config file")
        sys.exit(-1)

    interface_manager.set_default_hanlder(worker)
    signal.signal(signal.SIGINT, sigint_handler)
    app.exec_()
    interface_manager.destroy()
