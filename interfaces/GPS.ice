#ifndef GPS_ICE
#define GPS_ICE

module RoboCompGPS
{
  interface GPS
  {
    bool getData(out float latitude, out float longitude, out float altitude);
    bool getPos(out float x, out float y, out float z);
    bool getUTMData(out int xzone, out string yzone, out double eastering, out double northing);
    void resetPos();
  };
};

#endif
