# Laser.idsl

Laser.idsl provides a simple interface to access laser devices:

    import "DifferentialRobot.idsl";
    module RoboCompLaser
    {
        sequence<int> shortVector;
        struct LaserConfData
        {
           string driver;     // Underlying hardware: HokuyoURG/Gazebo
           string device;     // Laser device: hardware dependent
           int staticConf;    // 0 means it has a dynamic laser configuration
           int maxMeasures;   // Total number of possible measures (Laser specific)
           int maxDegrees;    // Angular measurement range degrees (Laser specific)
           int maxRange;      // Maximun distance measurable mm (Laser specific)
           int minRange;      // Minimun distance measurable mm (Laser specific)
           int iniRange;      // (0-totalRange) Initial measuring position
           int endRange;      // (0-totalRange) End measuring position
           int cluster;       // (0-99) Number of neighboor positions grouped
           int sampleRate;    // Adquisition period in msecs
           float angleRes;    // Angle resolution
           float angleIni;    // Initial angle
        };
        struct TData
        {
            float angle;
            float dist;
        };
       sequence<TData> TLaserData;
       interface Laser
       {
           TLaserData getLaserData();
           TLaserData getLaserAndBStateData(out RoboCompDifferentialRobot::TBaseState bState);
           LaserConfData getLaserConfData();
       };
    };


If you happen to be a hobbyist and built actual hobby robots then you would have come across or used proximity sensor for your bot. Laser interface is similar to this sensor. It is mainly used to calculate the distance between the nearest obstacle or an object.

To use the interface Laser first we define a vector to any variable, In this example it is ldata. ldata is a vector defined that stores the distances measured by the laser interface as structs,

    struct TData
    {
        float angle;
        float dist;
    };

And getLaserData() is the function that does the actual task of getting the distance measurements hence laser_proxy->getLaserData() in c++;
Thus the code in the specificworker of the calling component will be

	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData(); //c++
	ldata = laser_proxy.getLaserData(); //python

This stores the data that is generated using getLaserData into the vector ldata.




