#ifndef OBJECTDETECTOR_ICE
#define OBJECTDETECTOR_ICE

#include <Roimant.ice>

module RoboCompObjectdetector
{

  struct TImgPoint {
	int x;
	int y;
  };

  sequence<RoboCompRoimant::TRoi> TRoisIdList;
  sequence<TImgPoint> TImgPointsList;


  struct TObject {
	RoboCompRoimant::TRoi mainRoi;
	TRoisIdList roiList;
	float coveredArea;
	TImgPointsList limitPoints;

  };

  sequence<TObject> TObjectList;

  interface Objectdetector
  {

		idempotent void getObjectListAndStates(out TObjectList roiList, out RoboCompRoimant::TState state);

  };
};

#endif
