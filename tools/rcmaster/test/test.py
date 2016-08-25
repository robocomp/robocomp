import  pexpect, sys
from os import environ , path
from time import sleep
from copy import deepcopy , copy
import unittest

class Comp:
    def __init__(self, name, cmd):
        self.init_name = name
        self.name = self.init_name
        self.cmd = cmd
        self.staus = 0
        self.output = ''
        self.extra_params = ''
        self.dirty = False
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

    def start(self, confirm=False):
        ''''
        start the comp
        '''
        # if self.isRuning():
        #     self.stop()
        self.status = 0
        self.dirty = True
        self.output = ''
        fcmd = self.cmd + " --Ice.ProgramName="+ self.name + self.extra_params
        fcmd = fcmd.split(" ")
        # print fcmd
        self.process = pexpect.spawn(fcmd[0],fcmd[1:],timeout=None,env=environ.copy())
        if confirm:
            op = self.checkInOutput(["(Component|rcmaster) Started"],5,True)
            if op == 0:
                raise Exception(self.name +"cant start")

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
        print self.output
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

    def isRuning(self):
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
        self.dirty = False


class RCmasterTestCase(unittest.TestCase):
    def setUp(self):
        self.comps = dict()
        # cdirPath = path.dirname(path.abspath(__file__))
        
        self.comps["master"] = Comp("rcmaster","../src/rcmaster.py --Ice.Config=../etc/config")
        self.comps["client1"] = Comp("client1","clients/client1/src/client1.py --Ice.Config=clients/client1/etc/config")
        self.comps["client3"] = Comp("client3","clients/client3/src/client3.py --Ice.Config=clients/client3/etc/config")
        self.comps["client4"] = Comp("client4","clients/client4/src/client4.py --Ice.Config=clients/client4/etc/config")

    def getDuplComp(self, templateName, name):
        comp = copy(self.comps[templateName])
        comp.setName(name)
        return comp
        
    def tearDown(self):
        # stop all running comps
        for name, comp in self.comps.iteritems():
            if comp.dirty == True:
                comp.reset()

    # @unittest.skip("test skipping")
    def test_rcmaster_start(self):
        self.comps["master"].start(False)
        op1 = self.comps["master"].checkInOutput(["rcmaster Started"],10,True)
        self.assertTrue(op1,msg="master cant start")
        
    # @unittest.skip("test skipping")
    def test_multiple_master(self):
        self.comps["master"].start(True)

        master1 = self.getDuplComp("master","master1")
        master1.start()
        op1 = master1.checkInOutput(["Another instance of RCMaster is running"],10)
        self.assertTrue(op1,msg="multiple masters can run!!!")
        master1.stop()
    
    # @unittest.skip("test skipping")
    def test_component_registration(self):
        self.comps["master"].start(True)
        
        self.comps["client3"].start()
        op2 = self.comps["client3"].checkInOutput(["Component Started"],5)
        self.assertTrue(op2,msg="component cant start")
        op2 = self.comps["master"].checkInOutput(["name = client3"],5)
        self.assertTrue(op2,msg="component not registred")

    # @unittest.skip("test skipping")
    def test_multiple_same_interface(self):
        self.comps["master"].start(True)
        
        self.comps["client1"].start()
        op1 = self.comps["client1"].checkInOutput(["waiting for"],5)
        self.assertTrue(op1,msg="comp1 not waiting for test interface")
        
        comp31 = self.getDuplComp("client3","client31")
        comp31.start(True)
        op2 = self.comps["master"].checkInOutput(["name = client31"],5)
        self.assertTrue(op2,msg="client31 not registred")

        comp32 = self.getDuplComp("client3","client32")
        comp32.start(True)
        op2 = self.comps["master"].checkInOutput(["name = client32"],5)
        self.assertTrue(op2,msg="client32 not registred")

        op1 = comp31.checkInOutput(["hello from client1"],5)
        op2 = comp32.checkInOutput(["hello from client1"],5)
        self.assertTrue(op1,msg="cant recive msg from client1")

        comp31.stop()
        op1 = self.comps["client1"].checkInOutput(["waiting for"],10)
        self.assertTrue(op1,msg="comp1 not waiting for test interface after reset")
        comp32.stop()

    # @unittest.skip("test skipping")
    def test_caching(self):
        self.comps["master"].add_param("--rcmaster.cachettyl=2000")
        self.comps["master"].start(True)

        self.comps["client3"].start()
        op2 = self.comps["master"].checkInOutput(["name = client3"],5)
        self.assertTrue(op2,msg="component not registred")
        self.comps["client3"].stop()

        op1 = self.comps["master"].checkInOutput(["caching component  client3"],5)
        self.assertTrue(op1,msg="master not caching")
        
        op1 = self.comps["master"].checkInOutput(["removing  client3 from cache"], 10, True)
        self.assertTrue(op1,msg="master not removing cached client")



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
