
#ifndef RGB_ICE
#define RGB_ICE

module RobocompRGB{
	exception ImageException { string what; };
	
	sequence<int> rgbcolor;
	struct Image{
		rgbcolor red;
		rgbcolor green;
		rgbcolor blue;
	};
	
	interface RGB{
		void getImage(out Image color) throws ImageException;	
	};	
};

#endif


