#Interface - Laser

If you happen to be a hobbyist and built actual hobby robots then you would have come across or used proximity sensor for your bot. Laser interface is similar to this sensor. It is mainly used to calculate the distance between the nearest obstacle or an object.

To use the interface Laser first we define a vector to any variable, In this example it is ldata. ldata is a vector defined that stores the distances measured by the laser interface. getLaserData() is the function that does the actual task of getting the distance measurements hence laser_proxy->getLaserData();
Thus the code will be

	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();

This stores the data that is generated using getLaserData into the vector ldata.

	


