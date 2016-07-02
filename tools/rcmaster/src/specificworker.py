#
# Copyright (C) 2016 by YOUR NAME HERE
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

import sys, os, Ice, traceback, time, socket

from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP) < 1:
    print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
    sys.exit()

preStr = "-I" + ROBOCOMP + "/interfaces/ --all " + ROBOCOMP + "/interfaces/"
Ice.loadSlice(preStr + "RCMaster.ice")
from RoboCompRCMaster import *


# # redefine the hash funtion for component
# def generateCompId(self):
#   ''' 
#       generate unique id for the component
#   '''
#   hstr = self.name + host.__hash__()
#   for interface in self.interfaces:
#       hstr = hstr + interface.name
#   return hash(hstr)
# compData.__hash__ = generateCompId

# redefine hash function for interface
def newhash(self):
    return hash(self.name)


interfaceData.__hash__ = newhash

from rcmasterI import *


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)
        self.compdb = []
        self.compcache = dict()
        self.cache_ttyl = int(round(self.cache_ttyl / self.Period))
        self.savebit = False
        self.ic = Ice.initialize()

    def setParams(self, params):
        # try:
        #   par = params["InnerModelPath"]

        # except:
        #   traceback.print_exc()
        #   print "Error reading config params"
        return True

    @QtCore.Slot()
    def compute(self):
        print 'SpecificWorker.compute...'
        if self.savebit:
            self.savedb()

        # ping all componsnts and cache is necc
        for comp in self.compdb:
            for interface in comp.interfaces:
                proxy = interface.name + ':' + interface.protocol + " -h " + comp.host.publicIP + ' -p ' + str(
                    interface.port)
                basePrx = self.ic.stringToProxy(proxy)
                try:
                    basePrx.ice_ping()
                except ConnectionRefusedException:  # wbt other except @TODO
                    print "caching component ", comp.name
                    self.compdb.remove(comp)
                    self.compcache[comp] = self.cache_ttyl

        # invalidate cache based on ttyl
        for cachedComp, ttyl in self.compcache:
            ttyl = ttyl - 1
            if ttyl < 0:
                print "removing ", cachedComp.name, "from cache"
                del self.compcache[cachedComp]
        ## TODO
        # notify port changes or crash to the cmponents conected to crasehd one
        return True

    def checkComp(self, comp):
        # check valid name
        if comp.name == "":
            return False

        # check valid host
        remote_ip = ''
        if comp.host.hostName != '':
            try:
                remote_ip = socket.gethostbyname(comp.host.hostName)
            except socket.gaierror:
                print 'Hostname could not be resolved'
                return False

        if comp.host.privateIP == '' and remote_ip == '':
            return False
        elif comp.host.privateIP == '':
            comp.host.privateIP = remote_ip

        # check valid interfaces
        return True

    def get_open_port(self, portnum=0):
        '''
            get an open port for component
        '''
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(("", portnum))
        except socket.error, msg:
            print 'Cant assign port. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            return -1, msg
        port = s.getsockname()[1]
        s.close()
        return port, ""

    def savedb(self):
        pass

    def loaddb(self):
        pass

    ################################
    ############ Servents ##########
    ################################


    #
    # updateDb
    #
    def updateDb(self, components):
        for comp in components:
            self.registerComp(comp, False, False)

    #
    # registerComp
    #
    def registerComp(self, compInfo, monitor, assignPort=True):
        '''
        register a compoent and assagin a port to it
        @TODO monitor, multiple component for load balencing
        '''
        idata = interfaceData()

        # check if component is valid
        if not self.checkComp(compInfo):
            print "Not a valid component"
            raise InvalidComponent(compInfo, "")

        # check if entry already exists
        for comp in self.compdb:
            if comp.name == compInfo.name:
                print comp.name, 'already exists in host', comp.host.hostName
                raise DuplicateComponent(compInfo)

        if assignPort:

            # set the caches port for the components
            for cachedcomp in self.compcache.keys():
                if cachedcomp == compInfo:
                    compInfo.interfaces = cachedcomp.interfaces
                    break

            for interface in compInfo.interfaces:
                port, msg = self.get_open_port(interface.port)
                if port == -1:
                    print "couldnt assign cached port ", interface.port
                    port, msg = self.get_open_port()

                if port != -1:
                    interface.port = port
                else:
                    print "ERROR: Cant assign port to all interfaces"
                    raise PortAssignError(0, msg)

        self.compdb.append(compInfo)
        self.savebit = True
        print 'New component registred: ', comp.host, comp.name
        idata = compInfo.interfaces
        return idata

    #
    # getComps
    #
    def getComps(self, filter, block):
        # @TODO block
        if filter.name != '':
            return [x for x in self.compdb if x.name == filter.name]

        tempdb = self.compdb
        if filter.host.name != '':
            tempdb = [x for x in tempdb if x.host.name == filter.host.name]
        if filter.host.publicIP != '':
            tempdb = [x for x in tempdb if x.host.publicIP == filter.host.publicIP]
        if filter.host.privateIP != '':
            tempdb = [x for x in tempdb if x.host.privateIP == filter.host.privateIP]
        if len(filter.interfaces) != 0:
            tempdb = [x for x in tempdb if x.interfaces == filter.interfaces]
        return tempdb

    #
    # getComPort
    #
    def getComPort(self, compName, hostName, block):
        # @TODO block
        for comp in self.compdb:
            if comp.name == compName and comp.host.name == hostName:
                if len(comp.interfaces) != 1:
                    raise InvalidComponent
                return comp.interfaces[0].port
        raise ComponentNotFound

    #
    # flush
    #
    def flush(self, maindb):
        self.compdb = []
        if maindb:
            self.compcache = []
