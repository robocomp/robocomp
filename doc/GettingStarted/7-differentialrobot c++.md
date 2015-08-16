#Interface - DifferentialRobot

DIfferentialRobot offers command that helps it to program the motion of the bot.

#setSpeedBase(x, y)
This function is used to set the speed of the bot `x` in millimeters/sec and also to specify the rotating angle `y` in rad/sec for the bot to rotate.

	differentialrobot_proxy->setSpeedBase(x, y);

For example,

	differentialrobot_proxy->setSpeedBase(200, 0);

This line of code sets the bot speed to 200 mm/sec and rotating angle is 0 rad/sec construes that the bot will move forward at a speed of 200 mm/sec. Now that you know this function it becomes easier for us to make the bot to move in a square.

Consider another example,

  	differentialrobot_proxy->setSpeedBase(10, 1.5707);  //rotate 90 degrees at a speed of 10 mm/sec
  	usleep(1000000);  // delay 1 second

Here 1.5707 is nothing but 90 degrees in radians. Also keep in mind as it is rad/sec and not just radians. Changing the delay time to say 1.5 seconds would cause the bot to move for 135 degrees as 1.5707 rads/1.5sec.
