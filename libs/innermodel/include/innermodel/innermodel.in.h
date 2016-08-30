#ifndef INNERMODEL_H
#define INNERMODEL_H

// System includes
#include <stdexcept>
#include <stdint.h>
#include <typeinfo>

// Qt includes
#include <QHash>
#include <QMutexLocker>

// RoboComp includes
#include <qmat/qmat.h>
#include <qmat/qvec.h>
#include <qmat/qcamera.h>
#include <qmat/qrtmat.h>
#include <qmat/qfundamental.h>

// FCL
#define FCL_SUPPORT @FCL_SUPPORT_VALUE@

#if FCL_SUPPORT==1
#include <boost/shared_ptr.hpp>
#include <fcl/collision.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/ccd/motion.h>
#include <fcl/BV/BV.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/traversal/traversal_node_bvh_shape.h>
#include <fcl/traversal/traversal_node_bvhs.h>
typedef fcl::BVHModel<fcl::OBBRSS> FCLModel;
typedef boost::shared_ptr<FCLModel> FCLModelPtr;
#endif

#include <osg/TriangleFunctor>
#include <osg/io_utils>
#include <osg/Geode>
#include <osg/MatrixTransform>

using namespace RMat;

class InnerModel;
class InnerModelCamera;
class InnerModelDifferentialRobot;
class InnerModelOmniRobot;
class InnerModelIMU;
class InnerModelJoint;
class InnerModelTouchSensor;
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
	friend class InnerModelCamera;
	friend class InnerModelRGBD;
	friend class InnerModelReader;
public:
	struct AttributeType
	{
		QString type;
		QString value;
	};
	InnerModelNode(QString id_, InnerModelNode *parent_=NULL);
	virtual ~InnerModelNode()
	{
#if FCL_SUPPORT==1
		if (collisionObject!=NULL)
		{
			
			delete collisionObject;						
		}
		fclMesh.reset();
#endif
	}
	void treePrint(QString s, bool verbose=false);
	virtual void print(bool verbose) = 0;
	virtual void update() = 0;
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent) = 0;
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

	// FCLModel
	bool collidable;
#if FCL_SUPPORT==1
	FCLModelPtr fclMesh;
	fcl::CollisionObject *collisionObject;
#endif
};



class InnerModel
{
public:
	static bool support_fcl();
public:

	/// (Con/De)structors
	InnerModel();
	InnerModel(std::string xmlFilePath);
	InnerModel(const InnerModel &original);
	~InnerModel();
	friend class InnerModelReader;
	bool open(std::string xmlFilePath);
	bool save(QString path);	
	InnerModel* copy();

	/// Auto update method
	void update();
	void setUpdateRotationPointers(QString rotationId, float *x, float *y, float *z);
	void setUpdateTranslationPointers(QString translationId, float *x, float *y, float *z);
	void setUpdatePlanePointers(QString planeId, float *nx, float *ny, float *nz, float *px, float *py, float *pz);
	void setUpdateTransformPointers(QString transformId, float *tx, float *ty, float *tz, float *rx, float *ry, float *rz);
	void cleanupTables();

	/// Manual updat<< "hx" << "hy" << "hz" <<  "tx" << "ty" << "tz" << "rx" << "ry" << "rz" <<e method
	void updateTransformValuesS(std::string transformId, float tx, float ty, float tz, float rx, float ry, float rz, std::string parentId="")
	{
		updateTransformValues(QString::fromStdString(transformId), tx, ty, tz, rx, ry, rz, QString::fromStdString(parentId));
	}
	void updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId="");
	void updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId="");
	void updateRotationValues(QString transformId, float rx, float ry, float rz,QString parentId="");
	void updateJointValue(QString jointId, float angle, bool force=false);
	void updatePrismaticJointPosition(QString jointId, float position);
	void updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz);

	/// Model construction methods
	void setRoot(InnerModelNode *node);
	InnerModelTransform *newTransform(QString id, QString engine, InnerModelNode *parent, float tx=0, float ty=0, float tz=0, float rx=0, float ry=0, float rz=0, float mass=0);
	InnerModelJoint *newJoint(QString id, InnerModelTransform* parent, float lx = 0, float ly = 0, float lz = 0, float hx = 0, float hy = 0, float hz = 0,  float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, float min=-INFINITY, float max=INFINITY, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelTouchSensor *newTouchSensor(QString id, InnerModelTransform* parent, QString type, float nx = 0, float ny = 0, float nz = 0, float min=0, float max=INFINITY, uint32_t port=0);
	InnerModelPrismaticJoint *newPrismaticJoint(QString id, InnerModelTransform* parent, float min=-INFINITY, float max=INFINITY, float value=0, float offset=0, uint32_t port = 0, std::string axis = "z", float home=0);
	InnerModelDifferentialRobot *newDifferentialRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0, float noise=0., bool collide=false);
	InnerModelOmniRobot *newOmniRobot(QString id, InnerModelTransform* parent, float tx = 0, float ty = 0, float tz = 0, float rx = 0, float ry = 0, float rz = 0, uint32_t port = 0, float noise=0., bool collide=false);
	InnerModelCamera *newCamera(QString id, InnerModelNode *parent, float width, float height, float focal);
	InnerModelRGBD *newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, float noise, uint32_t port = 0, QString ifconfig="");
	InnerModelIMU *newIMU(QString id, InnerModelNode *parent, uint32_t port = 0);
	InnerModelLaser *newLaser(QString id, InnerModelNode *parent, uint32_t port = 0, uint32_t min=0, uint32_t max=30000, float angle = M_PIl, uint32_t measures = 360, QString ifconfig="");
	InnerModelPlane *newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx=0, float ny=0, float nz=0, float px=0, float py=0, float pz=0, bool collidable=0);
	InnerModelMesh *newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable=0);
	InnerModelMesh *newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable=0);
	InnerModelPointCloud *newPointCloud(QString id, InnerModelNode *parent);

	InnerModelTransform *getTransform(const QString &id);
	InnerModelJoint *getJoint(const QString &id);
	InnerModelJoint *getJoint(const std::string &id) { return getJoint(QString::fromStdString(id)); }
	InnerModelTouchSensor *getTouchSensor(const QString &id);
	InnerModelPrismaticJoint *getPrismaticJoint(const QString &id);
	InnerModelDifferentialRobot *getDifferentialRobot(const QString &id);
	InnerModelOmniRobot *getOmniRobot(const QString &id);
	InnerModelCamera *getCamera(QString id);
	InnerModelRGBD *getRGBD(QString id);
	InnerModelIMU *getIMU(QString id);
	InnerModelLaser *getLaser(QString id);
	InnerModelPlane *getPlane(const QString &id);
	InnerModelMesh *getMesh(const QString &id);
	InnerModelPointCloud *getPointCloud(const QString &id);

	/// Information retrieval methods
	QVec transform(const QString & destId, const QVec &origVec, const QString & origId);
	QVec transformS(const std::string & destId, const QVec &origVec, const std::string & origId)
	{
		return transform(QString::fromStdString(destId), origVec, QString::fromStdString(origId));
	}
	QVec transform6D(const QString &destId, const QVec &origVec, const QString & origId) { Q_ASSERT(origVec.size() == 6); return transform(destId, origVec, origId); }

	QVec transform6D(const QString &destId, const QString & origId) { return transform(destId, QVec::vec6(0,0,0,0,0,0), origId); }
	QVec transform(  const QString &destId, const QString & origId) { return transform(destId, QVec::vec3(0,0,0), origId); }
	QVec transformS( const std::string &destId, const std::string &origId)
	{
		return transform(QString::fromStdString(destId), QVec::vec3(0,0,0), QString::fromStdString(origId));
	}

	QVec rotationAngles(const QString & destId, const QString & origId);
	QVec project(QString reference, QVec origVec, QString cameraId);
	QVec project(const QString &cameraId, const QVec &origVec);
	QVec backProject(const QString &cameraId, const QVec &coord) ;//const;
	void imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS);
	QVec anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS);
	QVec imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString to);
	QVec projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist);
	QVec horizonLine(QString planeId, QString cameraId, float heightOffset=0.);

	/// Matrix transformation retrieval methods
	RTMat getTransformationMatrix(const QString &destId, const QString &origId);
	RTMat getTransformationMatrixS(const std::string &destId, const std::string &origId);
	QMat getRotationMatrixTo(const QString &to, const QString &from);
	QVec getTranslationVectorTo(const QString &to, const QString &from);
	QMat getHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	QMat getAffineHomographyMatrix(QString destCamera, QString plane, QString sourceCamera);
	QMat getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera);

	/// Misc
	void print(QString s="") { treePrint(s, true); }
	void treePrint(QString s="", bool verbose=false) { root->treePrint(QString(s), verbose); }

	/**
	 * @brief Returns the x,y,z coordinates of the center of reference of the robot. No angle is computed
	 *
	 * @return QVec
	 **/

	float getCameraFocal(const QString &cameraId ) const;
	int getCameraWidth( QString cameraId );
	int getCameraHeight(const QString &cameraId ) const;
	int getCameraSize(const QString &cameraId ) const;


	/// Stereo computations
	void updateStereoGeometry( const QString &firstCam, const QString & secondCam );
	QVec compute3DPointInCentral(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
	QVec compute3DPointInRobot(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right);
	QVec compute3DPointFromImageCoords(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);
	QVec compute3DPointFromImageAngles(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem);

	/// Laser stuff
	QVec laserTo(const QString &dest, const QString & laserId , float r, float alfa);

	/// Frustrum
	struct TPlane { QVec n; float d; };
	struct TFrustrum { TPlane left; TPlane top; TPlane right; TPlane down; TPlane near; TPlane far;};
	TFrustrum frustrumLeft, frustrumThird, frustrumRight;


	QList<QString> getIDKeys() {return hash.keys(); }
	InnerModelNode *getNode(const QString & id) const { if (hash.contains(id)) return hash[id]; else return NULL;}
	void removeSubTree(InnerModelNode *item, QStringList *l);
	void removeNode(const QString & id);
	void moveSubTree(InnerModelNode *nodeSrc, InnerModelNode *nodeDst);
	void getSubTree(InnerModelNode *node, QStringList *l);
	void getSubTree(InnerModelNode *node, QList<InnerModelNode *> *l);
	void computeLevels(InnerModelNode *node);
	/// Set debug level
	int debugLevel(int level=-1) { static int debug_level=0; if (level>-1) debug_level=level; return debug_level; }

	InnerModelNode *getRoot() { return root; }

	QString getParentIdentifier(QString id);
	std::string getParentIdentifierS(std::string id);

	// FCL related
	bool collidable(const QString &a);
	bool collide(const QString &a, const QString &b);
#if FCL_SUPPORT==1
	bool collide(const QString &a, const fcl::CollisionObject *obj);
#endif

	QMat jacobian(QStringList &listaJoints, const QVec &motores, const QString &endEffector);
	
protected:
	QMutex *mutex;
	InnerModelNode *root;
	QHash<QString, InnerModelNode *> hash;
	QHash<QPair<QString, QString>, RTMat> localHashTr;
	QHash<QPair<QString, QString>, QMat> localHashRot;
	
	void setLists(const QString &origId, const QString &destId);
	QList<InnerModelNode *> listA, listB;

};



class InnerModelTransform : public InnerModelNode
{
	friend class InnerModel;
	friend class InnerModelJoint;
	friend class InnerModelPointCloud;
	friend class InnerModelTouchSensor;
	friend class InnerModelLaser;
	friend class InnerModelDifferentialRobot;
	friend class InnerModelOmniRobot;
	friend class InnerModelPrismaticJoint;
	friend class InnerModelReader;
	InnerModelTransform(QString id_, QString engine_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float mass_, InnerModelNode *parent_=NULL);
public:
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void setUpdatePointers(float *tx_, float *ty_, float *tz_, float *rx_, float *ry_, float *rz_);
	void setUpdateTranslationPointers(float *tx_, float *ty_, float *tz_);
	void setUpdateRotationPointers(float *rx_, float *ry_, float *rz_);
	void update();
	void update(float tx_, float ty_, float tz_, float rx_, float ry_, float rz_);
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

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
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelJoint(QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_=-INFINITY, float max_=INFINITY, uint32_t port_=0,std::string axis_="z", float home_=0, InnerModelTransform *parent_=NULL);
public:
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void setUpdatePointers(float *lx_, float *ly_, float *lz_, float *hx_, float *hy_, float *hz_);
	void update();
	void update(float lx_, float ly_, float lz_, float hx_, float hy_, float hz_);
	float getAngle();
	float setAngle(float angle, bool force=false);
	QVec unitaryAxis();
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

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


class InnerModelTouchSensor : public InnerModelNode
{
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelTouchSensor(QString id_, QString stype, float nx_, float ny_, float nz_, float min_=-INFINITY, float max_=INFINITY, uint32_t port_=0, InnerModelNode *parent_=NULL);
public:
	void print(bool verbose) {}
	void save(QTextStream &out, int tabs) {}
	QVec getMeasure() { return value; }
	void update() {}
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	float nx, ny, nz;
	float min, max;
	float value;
	QString stype;
	uint32_t port;
};



class InnerModelPrismaticJoint : public InnerModelTransform
{
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelPrismaticJoint(QString id_, float min_, float max_, float val_, float offset_, uint32_t port_=0, std::string axis_="z", float home_=0, InnerModelTransform *parent_=NULL);
public:
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void update();
	float getPosition();
	float setPosition(float v);
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	float value, offset;
	float min, max;
	float home;
	uint32_t port;
	std::string axis;
};



class InnerModelDifferentialRobot : public InnerModelTransform
{
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelDifferentialRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_=0, float noise=0, bool collide=false, InnerModelTransform *parent_=NULL);
public:
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	uint32_t port;
	float noise;
	bool collide;
};

class InnerModelOmniRobot : public InnerModelTransform
{
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelOmniRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_=0, float noise=0, bool collide=false, InnerModelTransform *parent_=NULL);
public:
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	uint32_t port;
	float noise;
	bool collide;
};



class InnerModelPlane : public InnerModelNode
{
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelPlane(QString id_, QString texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, bool collidable, InnerModelNode *parent_=NULL);
public:
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void setUpdatePointers(float *nx_, float *ny_, float *nz_, float *px_, float *py_, float *pz_);
	void update();
	void update(float nx_, float ny_, float nz_, float px_, float py_, float pz_);
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	QVec normal, point;
	QString texture;
	float width, height, depth;
	int repeat;
	float *nx, *ny, *nz;
	float *px, *py, *pz;
};



class InnerModelCamera : public InnerModelNode
{
	friend class InnerModel;
	friend class InnerModelRGBD;
	friend class InnerModelReader;
	InnerModelCamera(QString id_, float width_, float height_, float focal_, InnerModelNode *parent_=NULL);
public:
	void print(bool verbose);
	void save(QTextStream &out, int tabs);
	void update();
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	Cam camera;
	float width, height, focal;
	float getWidth()  const { return width; }
	float getHeight() const { return height; }
	float getFocal()  const { return focal; }
	float getSize()   const { return getWidth()*getHeight(); }
};



class InnerModelRGBD : public InnerModelCamera
{
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelRGBD(QString id_, float width, float height, float focal, float _noise, uint32_t _port, QString _ifconfig, InnerModelNode *parent_=NULL);
	void save(QTextStream &out, int tabs);
public:
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	float noise;
	uint32_t port;
	QString ifconfig;
};



class InnerModelIMU : public InnerModelNode
{
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelIMU(QString id_, uint32_t _port, InnerModelNode *parent_=NULL);
public:
	void save(QTextStream &out, int tabs);
	void print(bool verbose);
	void update();
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	uint32_t port;
};



class InnerModelLaser : public InnerModelNode
{
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelLaser(QString id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, QString _ifconfig, InnerModelNode *parent_=NULL);
public:
	void save(QTextStream &out, int tabs);
	void print(bool verbose);
	void update();
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

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
protected:
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelMesh(QString id_, QString meshPath_, float scale, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable, InnerModelNode *parent_=NULL);
	InnerModelMesh(QString id_, QString meshPath_, float scalex_, float scaley_, float scalez_, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable, InnerModelNode *parent_=NULL);
public:

	void save(QTextStream &out, int tabs);
	void print(bool verbose);
	void update();
	void setScale(float x, float y, float z);
	bool normalRendering() const;
	bool wireframeRendering() const;
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);

public:
	RenderingModes render;
	QString meshPath;
	float scalex, scaley, scalez;
	float tx, ty, tz;
	float rx, ry, rz;
};



class InnerModelPointCloud : public InnerModelNode
{
	friend class InnerModel;
	friend class InnerModelReader;
	InnerModelPointCloud(QString id_, InnerModelNode *parent_=NULL);
public:
	void save(QTextStream &out, int tabs);
	void print(bool verbose);
	void update();
	virtual InnerModelNode *copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent);
};





#if FCL_SUPPORT==1
struct IncludeTrianglesInFCL_functor
{
	IncludeTrianglesInFCL_functor()
	{
		vertices = NULL;
		triangles = NULL;
	}

	std::vector<fcl::Vec3f> *vertices;
	std::vector<fcl::Triangle> *triangles;
	osg::Matrix tm;

	void set(std::vector<fcl::Vec3f> *vertices_, std::vector<fcl::Triangle> *triangles_, osg::Matrix transformMatrix)
	{
		vertices = vertices_;
		triangles = triangles_;
		tm = transformMatrix;
	}
	void clear()
	{
		if (vertices == NULL or triangles == NULL)
		{
			fprintf(stderr, "IncludeTrianglesInFCL_functor not initialized!\n");
			throw "IncludeTrianglesInFCL_functor not initialized!";
		}
		vertices->clear();
		triangles->clear();
	}
	void operator() (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3, bool /* treatVertexDataAsTemporary */)
	{
		if (vertices == NULL or triangles == NULL)
		{
			fprintf(stderr, "IncludeTrianglesInFCL_functor not initialized!\n");
			throw "IncludeTrianglesInFCL_functor not initialized!";
		}
		osg::Vec3 v1p = tm * v1;
		osg::Vec3 v2p = tm * v2;
		osg::Vec3 v3p = tm * v3;
		vertices->push_back(fcl::Vec3f(v1p.x(), v1p.y(), v1p.z()));
		vertices->push_back(fcl::Vec3f(v2p.x(), v2p.y(), v2p.z()));
		vertices->push_back(fcl::Vec3f(v3p.x(), v3p.y(), v3p.z()));
		triangles->push_back(fcl::Triangle(vertices->size()-3, vertices->size()-2, vertices->size()-1));
	}
};


class CalculateTriangles : public osg::NodeVisitor
{
public:
	CalculateTriangles(std::vector<fcl::Vec3f> *vertices_, std::vector<fcl::Triangle> *triangles_) : NodeVisitor( NodeVisitor::TRAVERSE_ALL_CHILDREN )
	{
		vertices = vertices_;
		triangles = triangles_;
		transformMatrix.makeIdentity();
	}

	virtual void apply(osg::Geode &geode)
	{
		// Use an OSG triangle functor to gather the vertices and triangles
		std::vector<fcl::Vec3f> vs;
		std::vector<fcl::Triangle> ts;
		osg::TriangleFunctor<IncludeTrianglesInFCL_functor> tri;
		tri.set(&vs, &ts, transformMatrix);
		tri.clear();
		int D = geode.getNumDrawables();
		for (int d=0; d<D; d++)
		{
			geode.getDrawable(d)->accept(tri);
		}
		// Append new points
		vertices->insert(vertices->end(), vs.begin(), vs.end());
		for (uint t=0; t<ts.size(); t++)
		{
			ts[t].set(ts[t][0]+triangles->size(), ts[t][1]+triangles->size(), ts[t][2]+triangles->size());
		}
		triangles->insert(triangles->end(), ts.begin(), ts.end());
		// recursion
		traverse( geode );
	}

	virtual void apply(osg::MatrixTransform &node)
	{
		// update matrix
		transformMatrix *= node.getMatrix();
		// recursion
		traverse( node );
	}
protected:
	// Pointers that we will be given and that we have to fill with the output data
	std::vector<fcl::Vec3f> *vertices;
	std::vector<fcl::Triangle> *triangles;
	// Transformation matrix
	osg::Matrix transformMatrix;
};

#endif




#endif
