/** \mainpage RoboComp Interfaces: Octomap.ice
 *
 * \section intro_sec Introduction
* Interface for octomapComp.
*
*
*
*    PORT  <br>
*/
#ifndef OCTOMAP_ICE
#define OCTOMAP_ICE

/** \namespace RoboCompOctomap
  *@brief Name space Octomap
  */
module RoboCompOctomap
{
  struct OctoMapParams
  {
    string InnerModel;
    double Precision;
  };

  /** \interface Octomap
  *@brief interface Octomap
  */
  interface Octomap
  {
    void setResolution(double res);
    double getResolution();
  };
};

#endif
