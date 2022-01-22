# Control the directions of the bot using keyboard

For this python component. Import DifferentialRobot and build the component using robocompdsl. This has been explained in detail in the previous tutorials. Next write the algorithm in specificworker.py. The algorithm is as follows

1. Get the input data from the keyboard
2. Check the key and perform actions accordingly
3. If same key is pressed again then increase the parameter

To implement this first initialize the code with the following in specific worker.py

	screen = curses.initscr()  //get the curses screen window
 	curses.noecho()            //turn off input echoing
 	curses.cbreak()            //respond to keys immediately (don't wait for enter)
	screen.keypad(True)        //map arrow keys to special values

Define a parameter key which will get the input form the keyboard. If in case the key pressed is up arrow (KEY_UP) then increase the adv by 20 (arbitrary) and by using differentialrobot's setSpeedBase function control the bot. adv and rot are intialized to 0 initially. This is similar to other cases. The code is as follows,

	def compute(self):
            try:
                key = screen.getch()

                if key == curses.KEY_UP:
                    self.adv = self.adv + 20
                    screen.addstr(5, 0, 'up: '+ str(self.adv)+ ' : ' + str(self.rot))
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
                elif key == curses.KEY_DOWN:
                    self.adv = self.adv - 20
                    screen.addstr(5, 0, 'down: '+ str(self.adv)+ ' : ' + str(self.rot))
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
                elif key == curses.KEY_LEFT:
                    self.rot = self.rot - 0.1;
                    screen.addstr(5, 0, 'left: '+ str(self.adv)+ ' : ' + str(self.rot))
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
                elif key == curses.KEY_RIGHT:
                    self.rot = self.rot + 0.1;
                    screen.addstr(5, 0, 'right: '+ str(self.adv)+ ' : ' + str(self.rot))
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
                elif key == ord(' '):
                    self.rot = 0
                    self.adv = 0
                    screen.addstr(5, 0, 'stop: '+ str(self.adv)+ ' : ' + str(self.rot))
                    self.differentialrobot_proxy.setSpeedBase(self.adv, self.rot)
               	elif key == ord('q'):
		    curses.endwin()
		    sys.exit()
            except Ice.Exception as e:
		curses.endwin()
                traceback.print_exc()
                print(e)
            return True

Save the code and simulate the innermodel simpleworld.xml and run the component by executing,

	python src/<componentname>.py --Ice.Config=etc/config

The code for the entire component can be found [here](https://github.com/robocomp/robocomp-robolab/tree/master/components/hardware/external_control/keyboardrobotcontroller)

