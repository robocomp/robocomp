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

import sys, os, Ice, traceback, time

from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
    print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
    sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"RCMaster.ice")
from RoboCompRCMaster import *
Ice.loadSlice(preStr+"ASR.ice")
from RoboCompASR import *
Ice.loadSlice(preStr+"Test.ice")
from RoboCompTest import *


from asrI import *

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)

    def setParams(self, params):
        #try:
        #    par = params["InnerModelPath"]
        #    innermodel_path=par.value
        #    innermodel = InnerModel(innermodel_path)
        #except:
        #    traceback.print_exc()
        #    print "Error reading config params"
        return True

    @QtCore.Slot()
    def compute(self):
        print '\nSpecificWorker.compute...'
        try:
            self.proxyData["testProxy"].printmsg("hello from client1")
        except Ice.SocketException:
            self.waitForComp("client3","test")
        return True

    def waitForComp(self, compName, interfaceName, updateAll=False):
        '''
        to be called when an interface call fails and need to wait for
         componet hosting that interface

        compName - name of the component to wait for
        interfaceName - name of the interface failed
        updateAll - update all proxies hosted by this component
        '''
        self.timer.stop()
        ic = Ice.initialize()
        
        dg = str(self.proxyData[str(interfaceName)+"Proxy"].ice_datagram())
        host = dg[ dg.find('-h')+3:dg.find("-p")-1]
        
        while True:
            try:
                interfaces = self.proxyData["rcmasterProxy"].getComp(compName,host)
                for iface in interfaces:
                    if updateAll or iface.name == interfaceName:
                        basePrx = ic.stringToProxy(iface.name+":"+iface.protocol+" -h "+host+" -p "+str(iface.port))
                        self.proxyData[str(iface.name+"Proxy")] = getattr(self.castData[compName],"testPrx").checkedCast(basePrx)

            except ComponentNotFound:
                print 'waiting for '+ compName
                time.sleep(3)
            except Ice.SocketException:
                # rcmaster has not detected the crash wait for some time
                time.sleep(3)
            except Ice.Exception:
                print 'Cannot connect to the remote object '+compName
                traceback.print_exc()
                time.sleep(3)
            else:
                self.timer.start(self.Period)
                break

    #
    # listenWav
    #
    def listenWav(self, path):
        #
        # YOUR CODE HERE
        #
        pass


    #
    # listenVector
    #
    def listenVector(self, audio):
        #
        # YOUR CODE HERE
        #
        pass


    #
    # resetPhraseBuffer
    #
    def resetPhraseBuffer(self):
        #
        # YOUR CODE HERE
        #
        pass


    #
    # getLastPhrase
    #
    def getLastPhrase(self):
        ret = string()
        #
        # YOUR CODE HERE
        #
        return ret


    #
    # phraseAvailable
    #
    def phraseAvailable(self):
        ret = bool()
        #
        # YOUR CODE HERE
        #
        return ret
