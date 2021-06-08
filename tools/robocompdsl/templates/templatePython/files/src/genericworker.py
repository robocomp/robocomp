#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) ${year} by YOUR NAME HERE
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

import sys, Ice, os

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
    print('$$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'

Ice.loadSlice("-I ./src/ --all ./src/CommonBehavior.ice")
import RoboCompCommonBehavior

${ui_import}



class GenericWorker(${qt_class_type}):

    ${qt_kill_signal}
    ${statemachine_signals}

    def __init__(self, mprx):
        super(GenericWorker, self).__init__()

        ${requires_proxies}
        ${publishes_proxies}
        ${gui_setup}

        ${statemachine_states_creation}

    ${statemachine_slots_creation}

    ${kill_yourself_method}

    ${set_period_method}
