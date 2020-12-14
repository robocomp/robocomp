import threading
import sys, Ice

Ice.loadSlice("-I ./src/ --all ./DSRGetID.ice")
import RoboCompDSRGetID

class DSRGetIDI(RoboCompDSRGetID.DSRGetID):
    _counter = 0
    _lock = threading.Lock()

    def getID(self, current=None):
        with self._lock:
            self._counter += 1
            #print(self._counter)
            return self._counter

with Ice.initialize(sys.argv) as communicator:
    adapter = communicator.createObjectAdapterWithEndpoints("dsrgetid:tcp", "default -p 11000")
    object = DSRGetIDI()
    adapter.add(object, communicator.stringToIdentity("dsrgetid"))
    adapter.activate()
    communicator.waitForShutdown()