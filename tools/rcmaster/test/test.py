import  pexpect, sys
from os import environ , path
from time import sleep
from copy import deepcopy
import unittest

class Comp:
    def __init__(self, name, cmd):
        self.init_name = name
        self.name = self.init_name
        self.cmd = cmd
        self.staus = 0
        self.output = ''
        self.extra_params = ''
        
        self.process = pexpect.spawn('true')
        self.process.kill(0)
    
    def setName(self, name):
        '''
        set the name of component
        '''
        self.name = name

    def add_param(self, param, append=True):
        '''
        add an extra param to be passed while calling the component
        '''
        if append:
            self.extra_params = self.extra_params + " " + param
        else:
            self.extra_params = param

    def start(self):
        ''''
        start the comp
        '''
        self.status = 0
        self.output = ''
        fcmd = self.cmd + " --Ice.ProgramName="+ self.name + self.extra_params
        fcmd = fcmd.split(" ")
        # print fcmd
        self.process = pexpect.spawn(fcmd[0],fcmd[1:],timeout=None,env=environ.copy()) 

    def stop(self):
        '''
        stop the comp
        '''
        self.process.sendcontrol('c')
        self.process.kill(0)

    def getOutput(self):
        '''
        get all output till now
        '''
        l = self.process.buffer
        self.output = self.output + str(l)
        return self.output

    def getCurrentOutput(self):
        '''
        get top output line in buffer
        '''
        line = str(self.process.readline())
        self.output = self.output + line
        return line.strip()

    def checkInOutput(self, texts, timeOut=30, pattern=False):
        '''
        wait <timeout> seconds untill the text appears
        @ pattern : given test is an pattern
        @ texts : list or string of pattern/texts to matched
        '''
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
        '''
        get comp running status
        '''
        self.status = self.process.isalive()
        return self.status

    def reset(self):
        '''
            reset to the init state
        '''
        self.stop()
        self.extra_params = ''
        self.setName(self.init_name)


class RCmasterTestCase(unittest.TestCase):
    def setUp(self):
        self.comps = dict()
        # cdirPath = path.dirname(path.abspath(__file__))
        
        self.comps["master"] = Comp("rcmaster","../src/rcmaster.py --Ice.Config=../etc/config")
        self.comps["comp1"] = Comp("client1","clients/client1/src/client1.py --Ice.Config=clients/client1/etc/config")
        self.comps["comp3"] = Comp("client3","clients/client3/src/client3.py --Ice.Config=clients/client3/etc/config")
        self.comps["comp32"] = Comp("client32","clients/client3/src/client3.py --Ice.Config=clients/client3/etc/config")
        self.comps["comp4"] = Comp("client4","clients/client4/src/client4.py --Ice.Config=clients/client4/etc/config")

    def tearDown(self):
        # stop all running comps
        for name, comp in self.comps.iteritems():
            if comp.getStatus() == True:
                comp.stop()


    # @unittest.skip("test skipping")
    def test_rcmaster_start(self):
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

    # @unittest.skip("test skipping")
    def test_multiple_same_interface(self):
        self.comps["master"].start()
        op1 = self.comps["master"].checkInOutput(["Starting rcmaster"],10)
        self.assertTrue(op1,msg="master cant start")
        
        self.comps["comp1"].start()
        op1 = self.comps["comp1"].checkInOutput(["waiting for"],10)
        self.assertTrue(op1,msg="comp1 not waiting for test interface")
        
        self.comps["comp3"].setName("client31")
        self.comps["comp3"].start()
        op1 = self.comps["comp3"].checkInOutput(["Component Started"],5)
        self.assertTrue(op1,msg="component cant start")
        op2 = self.comps["master"].checkInOutput(["name = client31"],5)
        self.assertTrue(op2,msg="client31 not registred")

        self.comps["comp32"].start()
        op1 = self.comps["comp32"].checkInOutput(["Component Started"],5)
        self.assertTrue(op1,msg="component cant start")
        op2 = self.comps["master"].checkInOutput(["name = client32"],5)
        self.assertTrue(op2,msg="client32 not registred")

        op1 = self.comps["comp3"].checkInOutput(["hello from client1"],10)
        op2 = self.comps["comp32"].checkInOutput(["hello from client1"],10)
        self.assertTrue(op1,msg="cant recive msg from comp1")

        self.comps["comp3"].stop()
        op1 = self.comps["comp1"].checkInOutput(["waiting for"],10)
        self.assertTrue(op1,msg="comp1 not waiting for test interface after reset")
    
    def test_caching(self):
        # self.comps["master"].start()
        # op1 = self.comps["master"].checkInOutput(["Starting rcmaster"],10)
        # self.assertTrue(op1,msg="master cant start")
        
        # self.comps["comp3"].start()
        # op2 = self.comps["comp3"].checkInOutput(["Component Started"],5)
        # self.assertTrue(op2,msg="component cant start")
        # op2 = self.comps["master"].checkInOutput(["name = client3"],5)
        # self.assertTrue(op2,msg="component not registred")
        pass
        

if __name__ == '__main__':
    unittest.main()

    # comps = dict()
    # comps["master"] = Comp("rcmaster","../src/rcmaster.py --Ice.Config=../etc/config")
    # comps["master"].add_param("--rcmaster.cachettyl=2000")
    # comps["master"].start()
    # print "st ",comps["master"].getStatus()
    # op = comps["master"].checkInOutput(["Starting rcmaster"],10)
    # print op
    # comps["master"].stop()
