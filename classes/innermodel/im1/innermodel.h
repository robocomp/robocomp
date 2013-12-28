#ifndef INNERMODEL_H
#define INNERMODEL_H

// System includes
#include <stdexcept>
#include <stdint.h>
#include <typeinfo>

// Qt includes
#include <QHash>
#include <QMutexLocker>

// Robocomp includes
#include <QMat/qmat.h>
#include <QMat/qvec.h>
#include <QMat/qcamera.h>
#include <QMat/qrtmat.h>
#include <QMat/qfundamental.h>



using namespace RMat;

class InnerModel;
class InnerModelCamera;
class InnerModelDifferentialRobot;
class InnerModelIMU;
class InnerModelJoint;
class InnerModelLaser;
class InnerModelMesh;
class InnerModelNode;
class InnerModelPlane;
class InnerModelPointCloud;
class InnerModelPrismaticJoint;
class InnerModelReader;
class InnerModelRGBD;
class InnerModelTransform;



class InnerModelException : public std::runtime_error
{
public:
	InnerModelException(const std::string &reason);
};

class InnerModelNode : public RTMat
{
public:
	struct AttributeType {
		QString type;
		QString value;
	};
	InnerModelNode(QString id_, InnerModelNode *parent_=NULL);
	void treePrint(QString s, bool verbose=false);
	virtual void print(bool verbose) = 0;
	virtual void update() = 0;
	virtual void save(QTextStream &out, int tabs) = 0;
	void setParent(InnerModelNode *parent_);
	void addChild(InnerModelNode *child);
	void setFixed(bool f=true);
	bool isFixed();
	void updateChildren();

//protected:

	QString id;
	int level;
	bool fixed;
	InnerModelNode *parent;
	QList<InnerModelNode *> children;
	QHash<QString,AttributeType> attributes;
};



class InnerModel
{
public:

	/// (Con/De)structors
	InnerModel();
	InnerModel(std::string xmlFilePath);
	InnerModel(const InnerModel &original);
	~InnerModel();
	friend class InnerModelReader;
	bool save(QString path);
	
	/// Auto update method
	void update();
	void setUpdateRotationPointers(QString rotationId, float *x, float *y, float *z);
	void setUpdateTranslationPointers(QString translationId, float *x, float *y, float *z);
	void setUpdatePlanePointers(QString planeId, float *nx, float *ny, float *nz, float *px, float *py, float *pz);
	void setUpdateTransformPointers(QString transformId, float *tx, float *ty, float *tz, float *rx, float *ry, float *rz);
	void cleanupTables();
	
	/// Manual update method
	void updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId="");
	void updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId="");
	void updateRotationValues(QString transformId, float rx, float ry, float rz,QString parentId="");
	void updateJointValue(QString jointId, float angle);
	void updatePrismaticJointPosition(QString jointId, float position);
	void updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz);
	
	/// Model construction methods
	void setRoot(InnerModelNode *node);
	InnerModelTransform *newTransform(QString id, QString engine, InnerModelNode *parent, float tx=0, float ty=0, float tz=0, float rx=0, float ry=0, float rz=0, float mass=0);
	InnerModelJoint *newJoint(QString id, InnerModelTransform* parent, float lx = 0, float ly = 0, float lz = 0, float hx = 0, float hy = 0, float hz = 0,  float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, float min=-INFINITY, float max=INFINITY, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelPrismaticJoint *newPrismaticJoint(QString id, InnerModelTransform* parent, float min=-INFINITY, float max=INFINITY, float value=0, float offset=0, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelDifferentialRobot *newDifferentialRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0);
	InnerModelCamera *newCamera(QString id, InnerModelNode *parent, float width, float height, float focal);
	InnerModelRGBD *newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, float noise, uint32_t port = 0, QString ifconfig="");
	InnerModelIMU *newIMU(QString id, InnerModelNode *parent, uint32_t port = 0);
	InnerModelLaser *newLaser(QString id, InnerModelNode *parent, uint32_t port = 0, uint32_t min=0, uint32_t max=30000, float angle = M_PIl, uint32_t measures = 360, QString ifconfig="");
	InnerModelPlane *newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx=0, float ny=0, float nz=0, float px=0, float py=0, float pz=0);
	InnerModelMesh *newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz);
	InnerModelMesh *newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz);
	InnerModelPointCloud *newPointCloud(QString id, InnerModelNode *parent);
	
	InnerModelTransform *getTransform(const QString &id);
	InnerModelJoint *getJoint(const QString &id);
	InnerModelPrismaticJoint *getPrismaticJoint(const QString &id);
	InnerModelDifferentialRobot *getDifferentialRobot(const QString &id);
	InnerModelCamera *getCamera(QString id);
	InnerModelRGBD *getRGBD(QString id);
	InnerModelIMU *getIMU(QString id);
	InnerModelLaser *getLaser(QString id);
	InnerModelPlane *getPlane(const QString &id);
	InnerModelMesh *getMesh(const QString &id);
	InnerModelPointCloud *getPointCloud(const QString &id);
	
	/// Information retrieval methods
	QVec transform(const QString & destId, const QVec &origVec, const QString & origId);
	QVec project(QString reference, QVec origVec, QString cameraId);
	QVec backProject(const QString &cameraId, const QVec &coord) ;//const;
	void imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS);
	QVec anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS);
	QVec imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString to);
	QVec projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist); 
	QVec horizonLine(QString planeId, QString cameraId, float heightOffset=0.);
	
	/// Matrix transformation retrieval methods
	RTMat getTransformationMatrix(const QString &destId, const QString &origId);
	QMat getRotationMatrixTo(const QString &to, const QString &from);
	QVec getTranslationVectorTo(const QString &to, const QString &from);
	QMat getHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	QMat getAffineHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	QMat getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera);
	
	/// Misc
	void print(QString s="") { treePrint(s, true); }
	void treePrint(QString s="", bool verbose=false) { root->treePrint(QString(s), verbose); }

	/// Robex Base specific getters
	QVec robotToWorld(const QVec & vec);
	/**
	 * @brief Returns the x,y,z coordinates of the center of reference of the robot. No angle is computed
	 *
	 * @return QVec
	 **/
	QVec robotInWorld(); 
	float getBaseX();
	float getBaseZ();
	float getBaseAngle(); 
	float getBaseRadius(); 
	float getCameraFocal(const QString &cameraId ) const;
	int getCameraWidth( QString cameraId );
	int getCameraHeight(const QString &cameraId ) const;
	int getCameraSize(const QString &cameraId ) const;
	void getFundamental(float h[3][3]) const;
	QMat getFundamental() const;
	
	inline QVec getBaseCoordinates() { return transform("world", QVec::vec3(0,0,0), "base");};
	inline T getBaseLine() const { return 0;/*return rightCamTranslationToLeftCam().vectorNormL2(); */ }
	/**
	 * @brief Return the current pose (x,z,angle) of the robot in the world reference frame.
	 *
	 * @return QVec
	 **/
	QVec getBaseOdometry();
		
	/// Stereo computations
	void updateStereoGeometry( const QString &firstCam, const QString & secondCam );
	QVec compute3DPointInCentral(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
	QVec compute3DPointInRobot(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
	QVec compute3DPointFromImageCoords(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);
	QVec compute3DPointFromImageAngles(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);	
	
	/// Setters for model parameters
	
	/// Laser stuff
	QVec laserToWorld( const QString &laserId , const QVec &p) { return laserTo("world", laserId, p); }
	QVec laserToWorld( const QString &laserId , float r, float alfa) { return laserTo("world", laserId, r, alfa);	}
	QVec laserTo(const QString &dest, const QString & laserId , const QVec &p);
	QVec laserTo(const QString &dest, const QString & laserId , float r, float alfa);
	QVec laserToRefFrame(const QString & laserId , float r, float alpha, const QString & refFrame) { return laserTo(refFrame, laserId, r, alpha); }
	QVec worldToLaser( const QString & laserId , const QVec & p);
	QVec laserToBase( const QString & laserId , float r, float alfa);
	
	/// Frustrum
	struct TPlano { QVec n; float d; };
	struct TFrustrum { TPlano left; TPlano top; TPlano right; TPlano down; TPlano near; TPlano far;};
	TFrustrum frustrumLeft, frustrumThird, frustrumRight;
	
	/// Cloning
	InnerModel cloneFake( const QVec & basePose) const;
	
	QList<QString> getIDKeys() {return hash.keys(); }
	InnerModelNode *getNode(const QString & id) const { if (hash.contains(id)) return hash[id]; else return NULL;}
	void removeSubTree(InnerModelNode *item, QStringList *l);
	void removeNode(const QString & id)  { hash.remove(id); }
	void getSubTree(InnerModelNode *node, QStringList *l);
	/// Set debug level
	int debugLevel(int level=-1) { static int debug_level=0; if (level>-1) debug_level=level; return debug_level; } 
	
	InnerModelNode *getRoot() { return root; }

protected:
	QMutex *mutex;
	InnerModelNode *root;
	QHash<QString, InnerModelNode *> hash;
	QHash<QPair<QString, QString>, RTMat> localHashTr;
	QHash<QPair<QString, QString>, QMat> localHashRot;
	void setLists(const QString &origId, const QString &destId);
	QList<InnerModelNode *> listA, listB;
	QFundamental fundamental;
	QEssential essential;
	
};



class InnerModelTransform : public InnerModelNode
{
public:
	InnerModelTransform(QString id_, QString engine_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float mass_, InnerModelNode *parent_=NULL);
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void setUpdatePointers(float *tx_, float *ty_, float *tz_, float *rx_, float *ry_, float *rz_);
	void setUpdateTranslationPointers(float *tx_, float *ty_, float *tz_);
	void setUpdateRotationPointers(float *rx_, float *ry_, float *rz_);
	void update();
	void update(float tx_, float ty_, float tz_, float rx_, float ry_, float rz_);

public:
	float *tx, *ty, *tz;
	float *rx, *ry, *rz;
	float mass;
	float backtX, backtY, backtZ;
	float backrX, backrY, backrZ;
	bool gui_translation, gui_rotation;
	QString engine;
};



class InnerModelJoint : public InnerModelTransform
{
public:
	InnerModelJoint(QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_=-INFINITY, float max_=INFINITY, uint32_t port_=0,std::string axis_="z", float home_=0, InnerModelTransform *parent_=NULL);
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void setUpdatePointers(float *lx_, float *ly_, float *lz_, float *hx_, float *hy_, float *hz_);
	void update();
	void update(float lx_, float ly_, float lz_, float hx_, float hy_, float hz_);
	float getAngle();
	float setAngle(float angle);	
	QVec unitaryAxis();
	
public:
	float *lx, *ly, *lz;
	float *hx, *hy, *hz;
	float backlX, backlY, backlZ;
	float backhX, backhY, backhZ;
	float min, max;
	float home;
	uint32_t port;
	std::string axis;
};



class InnerModelPrismaticJoint : public InnerModelTransform
{
public:
	InnerModelPrismaticJoint(QString id_, float min_, float max_, float val_, float offset_, uint32_t port_=0, std::string axis_="z", float home_=0, InnerModelTransform *parent_=NULL);
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void update();
	float getPosition();
	float setPosition(float v);

public:
	float value, offset;
	float min, max;
	float home;
	uint32_t port;
	std::string axis;
};



class InnerModelDifferentialRobot : public InnerModelTransform
{
public:
	InnerModelDifferentialRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_=0, InnerModelTransform *parent_=NULL);

public:
	uint32_t port;
};



class InnerModelPlane : public InnerModelNode
{
public:
	InnerModelPlane(QString id_, QString texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, InnerModelNode *parent_=NULL);
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void setUpdatePointers(float *nx_, float *ny_, float *nz_, float *px_, float *py_, float *pz_);
	void update();
	void update(float nx_, float ny_, float nz_, float px_, float py_, float pz_);

public:
	QVec normal, point;
	QString texture;
	float width, height,depth;
	int repeat;
	float *nx, *ny, *nz;
	float *px, *py, *pz;
};



class InnerModelCamera : public InnerModelNode
{
public:
	InnerModelCamera(QString id_, float width_, float height_, float focal_, InnerModelNode *parent_=NULL);
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void update();

public:
	Cam camera;
	float width, height, focal;
};



class InnerModelRGBD : public InnerModelCamera
{
public:
	InnerModelRGBD(QString id_, float width, float height, float focal, float _noise, uint32_t _port, QString _ifconfig, InnerModelNode *parent_=NULL);
	void save(QTextStream &out, int tabs);

public:
	float noise;
	uint32_t port;
	QString ifconfig;
};



class InnerModelIMU : public InnerModelNode
{
public:
	InnerModelIMU(QString id_, uint32_t _port, InnerModelNode *parent_=NULL);
	void save(QTextStream &out, int tabs);
	void print(bool verbose);
	void update();

public:
	uint32_t port;
};



class InnerModelLaser : public InnerModelNode
{
public:
	InnerModelLaser(QString id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, QString _ifconfig, InnerModelNode *parent_=NULL);
	void save(QTextStream &out, int tabs);
	void print(bool verbose);
	void update();

public:
	uint32_t port;
	uint32_t min, max;
	float angle;
	uint32_t measures;
	QString ifconfig;
};



class InnerModelMesh : public InnerModelNode
{
public:
	enum RenderingModes { NormalRendering=0, WireframeRendering=1};
	
	InnerModelMesh(QString id_, QString meshPath_, float scale, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, InnerModelNode *parent_=NULL);
	InnerModelMesh(QString id_, QString meshPath_, float scalex_, float scaley_, float scalez_, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, InnerModelNode *parent_=NULL);
	void save(QTextStream &out, int tabs);
	void print(bool verbose);
	void update();
	void setScale(float x, float y, float z);
	bool normalRendering() const;
	bool wireframeRendering() const;
	
public:
	RenderingModes render;	
	QString meshPath;
	float scalex, scaley, scalez;
	float tx, ty, tz;
	float rx, ry, rz;
};



class InnerModelPointCloud : public InnerModelNode
{
public:
	InnerModelPointCloud(QString id_, InnerModelNode *parent_=NULL);
	void save(QTextStream &out, int tabs);
	void print(bool verbose);
	void update();
};

#endif
