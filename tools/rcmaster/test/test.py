import  pexpect, sys
from os import environ
from time import sleep

class Comp:
    
    def __init__(self, name, cmd):
        self.name = name
        self.cmd = cmd.split(" ")

    def start(self):
        self.process = pexpect.spawn(self.cmd[0],self.cmd[1:],timeout=None,env=environ.copy()) 

    def stop(self):
        self.process.sendcontrol('c')
        sleep(3)
        self.process.kill(0)

    def getOutput(self):
        # self.process.expect(pexpect.EOF)
        return self.process.before
        # pass

    def getCurrentOutput(self):
        line = self.process.readline()
        return line.strip()
        # sys.stdout.write(line)
        # sys.stdout.flush()
    

    def checkInOutput(self, texts, timeOut=30, pattern=False):
        try:
            if pattern:
                index = self.process.expect(texts, timeOut)
            else:
                index = self.process.expect_exact(texts, timeOut)
            if index == 0:
                return True
            elif index == 1:
                return False
        except pexpect.EOF:
            return False
        except pexpect.TIMEOUT:
            return False
        return False

class Tester:
    def __init__(self):

        self.master = Comp("rcmaster","../src/rcmaster.py --Ice.Config=../etc/config")
        self.comp1 = Comp("comp1","./clients/client1/src/client1.py --Ice.Config=clients/client1/etc/config")
        self.comp3 = Comp("comp3","./clients/client3/src/client3.py --Ice.Config=clients/client3/etc/config")
        self.comp4 = Comp("comp4","./clients/client4/src/client4.py --Ice.Config=clients/client4/etc/config")

    def run_tests(self):
        num_passed = []
        num_failed = []

        # test is rcmaster is starting # 1
        number = 1
        self.master.start()
        op = self.master.checkInOutput(["starting rcmaster"],10)
        if op:
            print "Test"+number+" passed"
            num_passed.append(number)
        else:
            print "Test"+number+" failed"
            print "Master Logs: "
            print self.master.getOutput()        
            num_failed.append(number)
        self.master.stop()

        # test if a compoennt is starting # 2
        number = 2
        self.master.start()
        op1 = self.master.checkInOutput(["Starting rcmaster"],10)
        self.comp3.start()
        op2 = self.comp3.checkInOutput(["Component Started"],5)
        if op1 and op2:
            print "Test"+number+" passed"
            num_passed.append(number)
        else:
            print "Test"+number+" failed"
            print "Master Logs: "
            print self.master.getOutput()        
            num_failed.append(number)
        self.master.stop()
        self.comp3.stop()



if __name__ == '__main__':
    tester = Tester()
    tester.run_tests()

    # master = Comp("rcmaster","../src/rcmaster.py --Ice.Config=../etc/config")
    # master.start()
    # sleep(3)
    # print master.getOutput()
    # master.stop()
