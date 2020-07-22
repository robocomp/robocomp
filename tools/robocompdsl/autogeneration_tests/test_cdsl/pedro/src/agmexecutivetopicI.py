#
#    Copyright (C) 2020 by YOUR NAME HERE
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

import sys, os, Ice

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
    raise RuntimeError('ROBOCOMP environment variable not set! Exiting.')


Ice.loadSlice("-I ./src/ --all ./src/AGMExecutiveTopic.ice")

from RoboCompAGMExecutiveTopic import *

class AGMExecutiveTopicI(AGMExecutiveTopic):
    def __init__(self, worker):
        self.worker = worker


    def edgeUpdated(self, modification, c):
        return self.worker.AGMExecutiveTopic_edgeUpdated(modification)

    def edgesUpdated(self, modifications, c):
        return self.worker.AGMExecutiveTopic_edgesUpdated(modifications)

    def selfEdgeAdded(self, nodeid, edgeType, attributes, c):
        return self.worker.AGMExecutiveTopic_selfEdgeAdded(nodeid, edgeType, attributes)

    def selfEdgeDeleted(self, nodeid, edgeType, c):
        return self.worker.AGMExecutiveTopic_selfEdgeDeleted(nodeid, edgeType)

    def structuralChange(self, w, c):
        return self.worker.AGMExecutiveTopic_structuralChange(w)

    def symbolUpdated(self, modification, c):
        return self.worker.AGMExecutiveTopic_symbolUpdated(modification)

    def symbolsUpdated(self, modifications, c):
        return self.worker.AGMExecutiveTopic_symbolsUpdated(modifications)
