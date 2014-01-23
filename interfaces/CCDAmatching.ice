
#ifndef CCDAMATCHING_ICE
#define CCDAMATCHING_ICE

/** \mainpage RoboComp Interfaces: Vision.ice
 *
 * \section intro_sec Introduction
* Interface for visionComp.  
*                                                                   
* Multiscale head tracking of image regions. 
*
*    PORT 10015 <br>   
*/

/** \namespace RoboCompVision
  *@brief Name space vision interface.
  */


module RoboCompCCDAmatching
{
  
  enum option { sift, point3D, gmm, cuba };
  struct matchingOption
  {
    option opt;
    double absC;
    double absL; 	
    double absThreshold;
    double relThreshold;
  };

  sequence<float> Point;
  sequence<Point> ListPoint;
  
  sequence<float> Row;
  sequence<Row> Covariance;
  sequence<Covariance> ListCovariance;

  struct Feature
  {
    Point p;
    Covariance c;
  };
  
  struct PairFeatures 
  {
	Feature f1;
	Feature f2;
	int weight;
  };
  
  sequence<PairFeatures> PairFeaturesList;
  
  interface CCDAmatching
  {
	PairFeaturesList matchWeighted( ListPoint f1, ListPoint f2, ListCovariance c1, ListCovariance c2, matchingOption matchOpt);
	PairFeaturesList matchNonWeighted( ListPoint f1, ListPoint f2, ListCovariance c1, ListCovariance c2, matchingOption matchOpt);
  };
};

#endif
