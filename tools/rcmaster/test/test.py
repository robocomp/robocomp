import  pexpect, sys
from os import environ
from time import sleep
import unittest

class Comp:
    def __init__(self, name, cmd):
        self.name = name
        self.cmd = cmd.split(" ")
        self.staus = 0
        self.output = ''
        self.process = pexpect.spawn('true')
        self.process.kill(0)

    def start(self):
        self.status = 0
        self.output = ''
        self.process = pexpect.spawn(self.cmd[0],self.cmd[1:],timeout=None,env=environ.copy()) 

    def stop(self):
        self.process.sendcontrol('c')
        self.process.kill(0)

    def getOutput(self):
        l = self.process.buffer
        self.output = self.output + str(l)
        return self.output

    def getCurrentOutput(self):
        line = str(self.process.readline())
        self.output = self.output + line
        return line.strip()

    def checkInOutput(self, texts, timeOut=30, pattern=False):
        try:
            if pattern:
                index = self.process.expect(texts, timeOut)
            else:
                index = self.process.expect_exact(texts, timeOut)
            
            self.output = self.output + self.process.before + self.process.after
            if index == 0:
                return True
            elif index == 1:
                return False
        except pexpect.EOF:
            return False
        except pexpect.TIMEOUT:
            return False
        return False

    def getStatus(self):
        self.status = self.process.isalive()
        return self.status


class RCmasterTestCase(unittest.TestCase):
    def setUp(self):
        self.comps = dict()

        self.comps["master"] = Comp("rcmaster","../src/rcmaster.py --Ice.Config=../etc/config")
        self.comps["comp1"] = Comp("comp1","./clients/client1/src/client1.py --Ice.Config=clients/client1/etc/config")
        self.comps["comp3"] = Comp("comp3","./clients/client3/src/client3.py --Ice.Config=clients/client3/etc/config")
        self.comps["comp4"] = Comp("comp4","./clients/client4/src/client4.py --Ice.Config=clients/client4/etc/config")

    def tearDown(self):
        # stop all running comps
        for name, comp in self.comps.iteritems():
            if comp.getStatus() == True:
                comp.stop()


    def test_rcaster_start(self):
        self.comps["master"].start()
        op1 = self.comps["master"].checkInOutput(["Starting rcmaster"],10)
        self.assertTrue(op1,msg="master cant start")

    # @unittest.skip("test skipping")
    def test_component_registration(self):
        self.comps["master"].start()
        op1 = self.comps["master"].checkInOutput(["Starting rcmaster"],10)
        self.assertTrue(op1,msg="master cant start")
        self.comps["comp3"].start()
        op2 = self.comps["comp3"].checkInOutput(["Component Started"],5)
        self.assertTrue(op2,msg="component cant start")
        op2 = self.comps["master"].checkInOutput(["name = client3"],5)
        self.assertTrue(op2,msg="component not registred")


if __name__ == '__main__':
    unittest.main()

    # comps = dict()
    # comps["master"] = Comp("rcmaster","../src/rcmaster.py --Ice.Config=../etc/config")
    # comps["master"].start()
    # print "st ",comps["master"].getStatus()
    # # print "h",comps["master"].getOutput()
    # op = comps["master"].checkInOutput(["Starting rcmaster"],10)
    # print op
    # comps["master"].stop()
