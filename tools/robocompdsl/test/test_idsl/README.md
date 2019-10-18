ICE GENERATION TESTING
============
the test_ice_genetaration.py 
/usr/bin/python3.6 /home/robolab/robocomp/tools/robocompdsl/test/test_idsl/test_ice_generation.py -d -v -f Robot /opt/robocomp/interfaces/IDSLs/
/xxx/auto_generated_ice_files/TrajectoryRobot2D.ice generation OK
Executing comparation for /xxx/auto_generated_ice_files/TrajectoryRobot2D.ice ... WAIT!
/opt/robocomp/interfaces/IDSLs/TrajectoryRobot2D.idsl comparation FAILED
/xxx/auto_generated_ice_files/OmniRobot.ice generation OK
Executing comparation for /xxx/auto_generated_ice_files/OmniRobot.ice ... WAIT!
/opt/robocomp/interfaces/IDSLs/OmniRobot.idsl comparation FAILED
/xxx/auto_generated_ice_files/RobotTrajectory.ice generation OK
Executing comparation for /xxx/auto_generated_ice_files/RobotTrajectory.ice ... WAIT!
/opt/robocomp/interfaces/IDSLs/RobotTrajectory.idsl comparation FAILED
/xxx/auto_generated_ice_files/DifferentialRobot.ice generation OK
Executing comparation for /xxx/auto_generated_ice_files/DifferentialRobot.ice ... WAIT!
/opt/robocomp/interfaces/IDSLs/DifferentialRobot.idsl comparation FAILED
4 idsl files found
4 ice files generated OK (0 failed)
0 are identical (4 are different)
	/xxx/auto_generated_ice_files/TrajectoryRobot2D.ice have been generated? TRUE Are equal? FALSE 3 different lines
		OLD <-> NEW
		["cpp:comparable"] <-> 
		["cpp:comparable"] <-> 
		["cpp:comparable"] <-> 
	/xxx/auto_generated_ice_files/OmniRobot.ice have been generated? TRUE Are equal? FALSE 4 different lines*
		OLD <-> NEW
		["cpp:comparable"] <-> 
		interfaceOmniRobot{ <-> interfaceOmniRobot
		moduleRoboCompOmniRobot{ <-> moduleRoboCompOmniRobot
		structTMechParams{ <-> structTMechParams
	/xxx/auto_generated_ice_files/RobotTrajectory.ice have been generated? TRUE Are equal? FALSE 3 different lines*
		OLD <-> NEW
		interfaceRobotTrajectory{ <-> interfaceRobotTrajectory
		moduleRoboCompRobotTrajectory{ <-> moduleRoboCompRobotTrajectory
		voidsetOdometer(Statestate); <-> voidsetOdometer(RoboCompDifferentialRobot::TBaseStatestate);
	/xxx/auto_generated_ice_files/DifferentialRobot.ice have been generated? TRUE Are equal? FALSE 2 different lines*
		OLD <-> NEW
		moduleRoboCompDifferentialRobot{ <-> moduleRoboCompDifferentialRobot
		structTMechParams{ <-> structTMechParams

The * indictate tha the old file is probably deprecated and generated with an old version of .ice generation.