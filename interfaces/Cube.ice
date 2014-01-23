#ifndef CUBE_ICE
#define CUBE_ICE

#include <Roimant.ice>
#include <Laser.ice>

module RoboCompCube
{

  enum ActiveCubeState{waiting, creatingNewModel, verifiyingRoom, verifiyingDoors, goingToNewRoom};

  struct TCell
  {
    float occupancy;
    float x;
    float y;
    float z;
  };
  
  struct Point3D
  {
    float x;
    float y;
    float z; 
  };
  
  struct Model
  {
	float xc;
	float yc;
	float angle;
	float width;
	float height;
  };
  
  struct Door
  {
	float xIni;
	float yIni;
	float xFin;
	float yFin;
  };


  sequence<TCell> TCellList;
  sequence<Point3D> Point3DList;
  sequence<Model> TModelList;
  sequence<Door> TDoorList;

  struct ModelGroup
  {
    TModelList models;
    int type;
  };

  sequence<ModelGroup> TModelGroupList;

  interface Cube
  {
  	// Adds region (TRoi) as defined in Roimant to the cube
  	void addRoi( RoboCompRoimant::TRoi roi);
  	
  	// Adds regions as a vector of (XYZ) points 
  	void addPoints( Point3DList points);
  	
  	//Adds a RoboCompLaser::TLaserData vector of measurements
  	void addLaserData( RoboCompLaser::TLaserData laserData);

	//Update the occupancy of the cells included in cellList
	void updateCellsState(TCellList cellList);
	
	// Returns complete cube
	TCellList getMap();
	
	// Returns the map of the model iModel
	TCellList getMapInModel(int iModel);

	//Returns the closest obstacle along given direction in World reference system along given direction dir expressed in radians locally wrt robot (0º ahead, negs to the left)
	Point3D closestObstacleAlongDir(float dir);
	
	//Returns the first free position along a given segment in World reference system
	Point3D firstFreePositionAlongSegment(float xOrig, float zOrig, float xDest, float zDest);

	//Returns the free trajectory in World reference system along given direction dir expressed in radians locally wrt robot (0º ahead, negs to the left)
	Point3DList freeTrajectoryAlongDir(float dir);

	bool freePathToGoalPosition(float x, float z, out Point3DList path);

	bool freeSegmentToTarget(float xOrig, float zOrig, float xTarget, float zTarget);

	bool isThereAnObstacleAtPosition(float xCenter, float zCenter, float rX, float rZ);

	//Returns a list with occupied cells inside circle of given radius
	TCellList closestObstacleInRadius(float radius);

	// Returns a list of occupied cells inside current frustrum
	TCellList closestObstacleInFrustrum();
	
	// Returns the current model
	Model getModel(int iModel);

	TModelGroupList getGroups();

	TModelList getModelList(out int currentModel);
	
	int getNumberOfModels();

	void resetCurrentModel();
	
	TDoorList getDoorList();

	TDoorList getDoorListInModel(int iModel);

	void projectToModel();

	void setOdometryCorrection(bool cO);

	void startControl();

	void stopControl();

	ActiveCubeState getState();
	
	void fixCurrentModel();

	
  };
};

#endif

