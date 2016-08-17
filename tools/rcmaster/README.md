```
```
# RCMaster
A name and port service for Robocomp


## Configuration parameters
As any other component, *rcmaster* needs a configuration file to start. In

    etc/config

you can find the configuration file. We can find there the following lines:

    rcmaster.Endpoints=tcp -h localhost -p 0
this defins the protocol anf the host on which the rcmaster should run

    rcmaster.dbPath=/var/tmp/RoboComp
this sets the path where the rcmaster database should be backed up (not used as of now)

    rcmaster.componentsToStart=rclogger
rcmaster will launch all the components specified here before starting

    rcmaster.cachettyl=20000
This sets the cache entrie's timeout in seconds. remember that the cache entries as updated in each period of the specific worker

## Starting the component

    cd  <RoboComp 's path>/tools/rcmaster

we can run the component as:

    src/rcmaster.py
also you can star the component with as custom config file as:

    src/rcmaster.py etc/config

## Testing RCMaster

After installing, go to the rcmaster root folder and you can run the automated tests to check if everything is running fine:

    cd  <RoboComp 's path>/tools/rcmaster/test
    python -m unitest -v rctest

## Manual Testing
If you wanna see rcmster in action. Fire up three terminals, start rcmaster in one

    cd  <RoboComp 's path>/tools/rcmaster
    src/rcmaster.py

you should see a message saying rcmaster started on some port, on another terminal let's start client4:

    cd  <RoboComp 's path>/tools/rcmaster/test/clients/client4
    src/client4.py

now you should see the client4 saying its waiting for test interface, lets start client3
which impliments test interface:

    cd  <RoboComp 's path>/tools/rcmaster/test/clients/client3
    src/client3.py

now you should be able to see client3 printing message from client4. While starting
and killing each components you should be able to see rcmaster printing relevent information
