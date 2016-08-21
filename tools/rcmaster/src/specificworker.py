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
        self.compdb = dict()
        self.compcache = dict()
        self.cache_ttyl = int(round(self.cache_ttyl / self.Period))
        self.savebit = False
        self.blocking_timeout_step = 5
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
        # print 'SpecificWorker.compute...'
        if self.savebit:
            self.savedb()

        # ping all componsnts and cache is necc
        for uid in self.compdb:
            comp = self.compdb[uid]
            if comp.status != CompStatus.Active:
                continue
            for interface in comp.interfaces:
                proxy = interface.name.lower() + ':' + interface.protocol + " -h " + comp.host.privateIP + ' -p ' + str(
                    interface.port)

                basePrx = self.ic.stringToProxy(proxy)
                # see if the component is up, else cache it
                try:
                    basePrx.ice_ping()
                except Ice.SocketException:  # wbt other except @TODO
                    print "caching component ", comp.name
                    comp.status = CompStatus.Stopped
                    self.compcache[uid] = self.cache_ttyl
                    self.show_stats()
                except Ice.ObjectNotExistException:
                    print "cant find proxy: ",proxy

        # invalidate cache based on ttyl
        for uid in self.compcache.keys():
            self.compcache[uid] = self.compcache[uid] - 1
            if self.compcache[uid] < 0:
                try:
                    print "removing ", self.compdb[uid].name, "from cache"
                    del self.compdb[uid]
                    del self.compcache[uid]
                    self.show_stats()
                except KeyError:
                    print "cache and db mismatch ",uid
                    continue
        return True

    def checkComp(self, comp):
        # check valid name
        if comp.name == "":
            return False

        # check valid host
        remote_ip = ''
        if comp.host.hostName != '':
            if comp.host.hostName=="localhost":
                comp.host.privateIP = '127.0.0.1'
            else:
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
        msg = ""
        port = -1
        if portnum != 0:
            if s.connect_ex(('127.0.0.1',portnum)) ==0 :
                msg = "Cant assign port, Alrady in use"
            else:
                port = portnum 
        else:
            try:
                s.bind(("", portnum))
            except socket.error, msg:
                msg = 'Cant assign port. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            else:
                port = s.getsockname()[1]

        s.close()
        return port, msg

    def generate_uid(self, comp):
        if not self.checkComp(comp):
            raise InvalidComponent(comp, "")
        uid = comp.name+comp.host.privateIP
        for interface in comp.interfaces:
            uid = uid+interface.name.lower()
        return str(hash(uid))

    def savedb(self):
        pass

    def loaddb(self):
        pass

    def show_stats(self):
        print "\n\n ---- STATS ----"
        print 'no of components registred :',(len(self.compdb)-len(self.compcache))
        # print self.compdb
        print 'no of components cached :',len(self.compcache)
        # print self.compcache
        print "\n------------------------------\n"
        
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
    def registerComp(self, compInfo, monitor, assignPort):
        '''
        register a compoent and assagin a port to it
        @TODO monitor, multiple component for load balencing
        '''
        idata = interfaceData()

        # check if component is valid
        compInfo.uid = self.generate_uid(compInfo)

        # check if entry already exists
        for uid in self.compdb:
            if uid == '':
                raise Exception("Invalid uid")
            if uid == compInfo.uid and self.compdb[uid].status == CompStatus.Active:
                print compInfo.name, 'already exists in host', compInfo.host.hostName
                raise DuplicateComponent(compInfo)

        if assignPort:

            # if its a cached component retrive data
            for uid in self.compcache.keys():
                if uid == compInfo.uid:
                    print "This is a cached component ..."
                    compInfo.interfaces = self.compdb[uid].interfaces
                    print "removing ", self.compdb[uid].name, "from cache"
                    del self.compcache[uid]
                    self.compdb[uid].status = CompStatus.Active
                    break

            for interface in compInfo.interfaces:
                print interface
                port, msg = self.get_open_port(interface.port)
                if port == -1:
                    print "couldnt assign cached port ", interface.port
                    port, msg = self.get_open_port()

                if port != -1:
                    interface.port = port
                else:
                    print "ERROR: Cant assign port to all interfaces"
                    raise PortAssignError(0, msg)

        # compInfo.uid = self.generate_uid(compInfo)
        compInfo.status = CompStatus.Active
        self.compdb[compInfo.uid] = compInfo
        self.savebit = True
        print 'New component registred: ', compInfo,'\n'
        self.show_stats()
        return compInfo.interfaces

    #
    # getComps
    #
    def getComps(self, filter):
        tempdb = self.compdb

        if filter.name != '':
            tempdb = {uid:comp for (uid,comp) in tempdb if comp.name == filter.name}
        if filter.host.name != '':
            tempdb = {uid:comp for (uid,comp) in tempdb if comp.host.name == filter.host.name}
        if filter.host.privateIP != '':
            tempdb = {uid:comp for (uid,comp) in tempdb if comp.host.privateIP == filter.host.privateIP}
        if filter.host.publicIP != '':
            tempdb = {uid:comp for (uid,comp) in tempdb if comp.host.publicIP == filter.host.publicIP}
        if len(filter.interfaces) != 0:
            tempdb = {uid:comp for (uid,comp) in tempdb if filter.interfaces in comp.interfaces}

        if len(tempdb) == 0:
            raise ComponentNotFound
        else:
            return tempdb

    #
    # getComPort
    #
    def getComPort(self, compName, privateIP):
        interfaces = self.getComp(compName, privateIP)
        if len(interfaces) != 1:
            raise InvalidComponent
        return interfaces[0].port

    #
    # getComp
    #
    def getComp(self, compName, privateIP):
        # if passed an hostname convert to ip
        privateIP=privateIP.strip()
        try:
            socket.inet_aton(privateIP)
        except socket.error:
            try:
                privateIP = socket.gethostbyname(privateIP)
            except socket.gaierror, err:
                print "cant resolve hostname :",privateIP,err
                raise InvalidComponent

        for comp in self.compdb.itervalues():
            if comp.status != CompStatus.Active:
                continue
            if comp.name == compName and comp.host.privateIP == privateIP:
                return comp.interfaces
        raise ComponentNotFound

    #
    # flush
    #
    def flush(self, maindb):
        print "Flushing the cache ..."
        for uid in self.compcache:
            self.compdb.pop(uid)
        self.compcache = dict()
        if maindb:
            self.compdb = dict()
            print "Flushing the mainDB ..."
        self.show_stats()


    def getStat(self, showNames):
        statStr = ''
        if showNames:
            statStr = 'registred : ',(len(self.compdb)-len(self.compcache)) + " ; "
            statStr = statStr + 'registred : ',len(self.compcache) + " ; "
        else:
            comps = " ".join(str(x.name) for x in self.compdb if x.status == CompStatus.Active)
            cached_comps = " ".join(str(x.name) for x in self.compdb if x.status != CompStatus.Active)
            statStr = 'registred : ' + comps + " ; "
            statStr = statStr + 'registred : ' + cached_comps + " ; "
        return statStr