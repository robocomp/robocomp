# ICE GENERATION TESTING

The `test_ice_genetaration.py` is intended to be able to partially **test the generation of .ice files from idsl,** specially after modifications on the internal code of robocompdsl tools. 
This testing let us check if the generation after modifications fails or the code generated is equal or similar to the old one.
Given a directory that content several .idsl files this script read those files, try to generate the expected .ice on a temporal directory and by default try to check the similitude with previous .ice files that are expected to be on the parent directory of the given .idsl source directory.

## Script options

    $ ./test_ice_generation.py -h
    usage: test_ice_generation.py	[-h] [-g] [-d] [-n] [-i INSTALLATION]  
								    [-f FILTER] [-v]  
									indir [outdir]  
      
    This application generate .ice files from idsl files to check generation and comparation  to old verions  
      
      
    positional arguments:  
    indir Input dir for idsl files  
    outdir Output dir for .ice and l.log files  
      
    optional arguments:  
    -h, --help show this help message and exit  
    -g, --generate-only Only the generation with robocompdsl is checked. No comparation is checked  
    -d, --dirty No cleaning is done after execution. All .ice and .log files will be left on the output dir.  
    -n, --dry-run Executing dry run. No remove, generation or comparation will be executed in dry run mode.  
	-i INSTALLATION, --installation INSTALLATION Option to change where the robocompdsl.py tool script will be found.  
    -f FILTER, --filter FILTER  
    Execute the checks only for .idsl files containing the FILTER string.
    -v, --visual gives a visual output of the different lines at the end  of the execution for each file.

## Execution example
This is an example of the execution of the script and it's output:

    $test_ice_generation.py -d -v -f Robot /opt/robocomp/interfaces/IDSLs/

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
      
    The * indictate that the old file is probably deprecated and generated with an old version of .ice generation.

The real output of the command is colored to be more easy to understand.
The options given to the script are:

 - -d: is for dirty execution. The resulting files from the execution are kept on the auto_generated_ice_files on the current dir (if no output
   dir is given.
   
 - -v: shows the lines different on each file as OLD <-> NEW comparation
 - -f Robot: limit the execution of the script to the files that contains 'Robot' in its name.
 - /opt/robocomp/interfaces/IDSLs/: this is the path where script expect to finde the .idsl files to check.

