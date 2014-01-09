/**
 * @file innermodel.h
 * @brief InnerModel is a library for modelling, simulating and handling of kinematic chains.
 * @author Robolab
 * @version 2.0
 * 
 * @section LICENSE
 * 
 * Copyright (C) 2010-2013 by RoboLab - University of Extremadura
 *
 * This file is part of RoboComp
 *
 * RoboComp is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoboComp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * @section DESCRIPTION
 * 
 * InnerModel.
 */

#pragma once

// System includes
#include <stdint.h>
#include <stdexcept>
#include <typeinfo>

// Qt includes
#include <QHash>
#include <QMutexLocker>
#include <QSet>
#include <QtGui>
#include <QtOpenGL/QGLWidget>

// OSG includes
#include <osg/Group>
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include <osg/Switch>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

// Bullet includes
#include <bullet/btBulletDynamicsCommon.h>
#include <osgbCollision/Utils.h>

// Robocomp includes
#include <QMat/QMatAll>



using namespace RMat;



/**
 * @brief InnerModel version 2.
 **/
namespace IM2 {

class InnerModel;
class Viewer;

class Node;
class Body;
class DifferentialRobot;

class Sensor;
class Camera;
class IMU;
class Laser;
class RGBD;

class Joint;
class BallJoint;
class GenericJoint;
class HingeJoint;
class PrismaticJoint;

class Primitive;
class Mesh;
class Plane;
class PointCloud;

extern QString xml_base_path;



// ------------------------------------------------------------------------------------------------
// Simple data types and free functions
// ------------------------------------------------------------------------------------------------

/**
 * @brief Default camera view angles.
 **/
enum CameraView { BACK_POV, FRONT_POV, LEFT_POV, RIGHT_POV, TOP_POV };



/**
 * @brief Optical parameters of a camera.
 **/
struct CameraParams {
	int width;     ///< Frame width, in pixels.
	int height;    ///< Frame height, in pixels.
	double focal;   ///< Focal length, in pixels.
	double fovy;   ///< Field of view in the Y axis, in radians.
	double aspect; ///< Frame aspect ratio, width/height.
	double znear;  ///< Distance of the near Z plane, in meters.
	double zfar;   ///< Distance of the far Z plane, in meters.
};



/**
 * @brief Available types of physical bodies.
 **/
enum BodyType {
	Static,    ///< Static rigid body of infinite mass.
	Dynamic,   ///< Dynamic rigid body controlled by the physics engine.
	Kinematic, ///< Dynamic rigid body controlled by its own will.
	Ghost      ///< Bodies without physical existence, used only for rendering.
};



/**
 * @brief Physical parameters of a rigid body.
 **/
struct BodyParams {
	BodyType type;     ///< Kind of body (static, dynamic, kinematic or ghost)
	float mass;        ///< Body mass in kilograms
	float friction;    ///< Friction coefficient
	float restitution; ///< Restitution coefficient, amount of energy conserved in collisions
	BodyParams() {
		type = Static;
		mass = 0.0f;
		friction = 0.0f;
		restitution = 0.0f;
	};
	BodyParams( const BodyType type_, const float mass_, const float friction_, const float restitution_ ) :
		type(type_), mass(mass_), friction(friction_), restitution(restitution_)
		{};
};



/**
 * @brief Available types of joints.
 **/
enum JointType {
	Free,  ///< Joint without control, e.g. a door hinge.
	Motor, ///< Motor controlled by Bullet.
	Manual ///< Motor manually controlled by the user.
};



/**
 * @brief Generic node attribute.
 **/
struct AttributeType {
	QString type;  ///< Datatype (possible values?)
	QString value; ///< Value, serialized as an string
};



/**
 * @brief Converts a left-handed QVec to a right-handed osg::Vec3
 *
 * @param vec Left-handed QMat vector
 * @return Right-handed OSG vector
 **/
osg::Vec3 QVecToOSGVec(const QVec &vec);



/**
 * @brief Converts a RGB color in HTML format to a osg::Vec4
 *
 * @param color RGB color as a HTML string
 * @return RGBA color, component range [0.0-1.0]
 **/
osg::Vec4 htmlStringToOsgVec4(QString color);



/**
 * @brief Converts RGBA color stored as an osg::Vec4 to HTML format
 *
 * @param color RGBA color as a tuple of floats
 * @return RGB color as a HTML string
 **/
QString osgVec4ToHtmlString(osg::Vec4 color);



/**
 * @brief Converts a left-handed QMat to a right-handed osg::Matrix
 *
 * @param mat Left-handed QMat transform matrix 
 * @return Right-handed OSG matrix
 **/
osg::Matrix QMatToOSGMat4( const RTMat& mat );



/**
 * @brief Converts a right-handed osg::Matrix to a left-handed QMat
 *
 * @param osm Input right-handed OSG matrix
 * @param mat Output left-handed QMat matrix
 * @return void
 **/
void OsgToQMat( const osg::Matrix& osm, RTMat& mat );



/**
 * @brief Converts a left-handed QMat to a right-handed btTransform
 *
 * @param mat Input left-handed QMat matrix
 * @param trf Output right-handed OSG matrix
 * @return void
 **/
void QMatToBullet( const RTMat& mat, btTransform& trf );



/**
 * @brief Converts a right-handed btTransform to a left-handed QMat
 *
 * @param trf Input right-handed Bullet matrix
 * @param mat Output left-handed QMat transform matrix
 * @return void
 **/
void BulletToQMat( const btTransform& trf, RTMat& mat );



/**
 * @brief Sets the camera position
 *
 * @param manipulator Manipulator associated with a camera
 * @param pov Desired point of view (left, right, front, back, top, bottom)
 * @return void
 **/
void setMainCamera( osgGA::TrackballManipulator *manipulator, CameraView pov );



// ------------------------------------------------------------------------------------------------
// Exception
// ------------------------------------------------------------------------------------------------

/**
 * @brief Runtime failure
 **/
class Exception : public std::runtime_error
{
public:
	/**
	 * @brief Constructor
	 *
	 * @param reason Message telling the user what went wrong
	 **/
	Exception( const std::string &reason );
};



// ------------------------------------------------------------------------------------------------
// InnerModel
// ------------------------------------------------------------------------------------------------

/**
 * @brief Kinematic tree of the world as seen by the robot.
 **/
class InnerModel
{
public:

	/// (Con/De)structors
	InnerModel();
	InnerModel(std::string xmlFilePath);
	InnerModel(const InnerModel &original);
	~InnerModel();
	
	// Serialization/deserialization methods
	bool load(const QString& file);
	bool save(const QString& path);
	
	/// Auto update method
	void initPhysics( osg::Group* graphicsScene );
	void updatePhysics( const float elapsed );
	void cleanupTables();
	void setRoot(Node *node);
	
	/// Manual update method
	void updateGenericJointValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId="");
	void updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId="");
	void updateRotationValues(QString transformId, float rx, float ry, float rz,QString parentId="");
	void updateHingeJointValue(QString jointId, float angle);
	void updatePrismaticJointPosition(QString jointId, float position);
	void updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz);
	
	/// Model construction methods
	GenericJoint *newGenericJoint(
		QString id,
		Node *parent,
		JointType jtype_,
		float tx=0, float ty=0, float tz=0,
		float rx=0, float ry=0, float rz=0,
		uint32_t port = 0 );
	BallJoint *newBallJoint(
		QString id,
		Body* parent,
		JointType jtype_,
		float lx = 0, float ly = 0, float lz = 0,
		float hx = 0, float hy = 0, float hz = 0,
		float tx = 0, float ty = 0, float tz = 0,
		float rx = 0, float ry = 0, float rz = 0,
		uint32_t port = 0 );
	HingeJoint *newHingeJoint(
		QString id,
		Body* parent,
		JointType jtype_,
		float tx = 0, float ty = 0, float tz = 0,
		float rx = 0, float ry = 0, float rz = 0,
		float min=-INFINITY, float max=INFINITY,
		std::string axis = "z",
		uint32_t port = 0 );
	PrismaticJoint *newPrismaticJoint(
		QString id,
		Body* parent,
		JointType jtype_,
		float tx = 0, float ty = 0, float tz = 0,
		float rx = 0, float ry = 0, float rz = 0,
		float min=-INFINITY, float max=INFINITY,
		float value=0,
		float offset=0,
		std::string axis = "z",
		uint32_t port = 0 );
	Plane *newPlane(
		QString id,
		Node *parent,
		QString texture,
		float width,
		float height,
		float depth,
		int repeat,
		float nx=0,
		float ny=0,
		float nz=0,
		float px=0,
		float py=0,
		float pz=0 );
	PointCloud *newPointCloud(
		QString id,
		Node *parent );
	Body* newBody(
		QString id,
		Node* parent,
		const RTMat& pose,
		const BodyParams& params );
	DifferentialRobot *newDifferentialRobot(
		QString id,
		Node* parent,
		float tx = 0,
		float ty = 0,
		float tz = 0,
		float rx = 0,
		float ry = 0,
		float rz = 0,
		uint32_t port = 0 );
	Camera *newCamera(
		QString id,
		Node *parent,
		float width,
		float height,
		float focal,
		uint32_t port = 0,
		QString ifconfig="");
	RGBD *newRGBD(
		QString id,
		Node *parent,
		float width,
		float height,
		float focal,
		uint32_t port = 0,
		QString ifconfig="" );
	IMU *newIMU(
		QString id,
		Node *parent,
		uint32_t port = 0);
	Laser *newLaser(
		QString id,
		Node *parent,
		uint32_t port = 0,
		uint32_t min=0,
		uint32_t max=30000,
		float angle = M_PIl,
		uint32_t measures = 360,
		QString ifconfig="");
	
	// Query methods
	GenericJoint *getGenericJoint( const QString& id ) const;
	BallJoint *getBallJoint( const QString& id ) const;
	HingeJoint *getHingeJoint( const QString& id ) const;
	PrismaticJoint *getPrismaticJoint( const QString& id ) const;
	
	Body* getBody( const QString& id ) const;
	DifferentialRobot *getDifferentialRobot( const QString& id ) const;
	
	Camera *getCamera( const QString& id ) const;
	RGBD *getRGBD( const QString& id ) const;
	IMU *getIMU( const QString& id ) const;
	Laser *getLaser( const QString& id ) const;
	
	Mesh *getMesh( const QString& id ) const;
	Plane *getPlane( const QString& id ) const;
	PointCloud *getPointCloud( const QString& id ) const;
	
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
	void print(QString s="");
	void treePrint(QString s="", bool verbose=false);

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
	int getCameraWidth( QString cameraId ) const;
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
	QVec laserToWorld( const QString &laserId , const QVec &p);
	QVec laserToWorld( const QString &laserId , float r, float alfa);
	QVec laserTo(const QString &dest, const QString & laserId , const QVec &p);
	QVec laserTo(const QString &dest, const QString & laserId , float r, float alfa);
	QVec laserToRefFrame(const QString & laserId , float r, float alpha, const QString & refFrame);
	QVec worldToLaser( const QString & laserId , const QVec & p);
	QVec laserToBase( const QString & laserId , float r, float alfa);
	
// 	/// Frustrum
// 	struct TPlano { QVec n; float d; };
// 	struct TFrustrum { TPlano left; TPlano top; TPlano right; TPlano down; TPlano near; TPlano far;};
// 	TFrustrum frustrumLeft, frustrumThird, frustrumRight;
	
	/// Cloning
	InnerModel cloneFake( const QVec & basePose) const;
	
	Node *getRoot();
	QList<QString> getIDKeys();
	Node *getNode(const QString & id) const;
	void removeNode(const QString & id);
	void removeSubTree(Node *item);
	void getSubTree(Node *node, QStringList& names) const;
	
	/// Set debug level
	int debugLevel(int level=-1);

protected:
	QMutex *mutex;
	Node *root;
	QHash<QString, Node *> hash;
	QHash<QPair<QString, QString>, RTMat> localHashTr;
	QHash<QPair<QString, QString>, QMat> localHashRot;
	QList<Node *> listA, listB;
	QFundamental fundamental;
	QEssential essential;
	void setLists(const QString &origId, const QString &destId);
	
	// Physics attributes
	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
};



// ------------------------------------------------------------------------------------------------
// Viewer
// ------------------------------------------------------------------------------------------------

/**
 * @brief Auxiliary class used for rendering the world
 **/
class Viewer : public osg::Switch
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param im Reference to the InnerModel tree to be rendered.
	 * @param root Tree node to be used as root.
	 * @param parent Scene graph parent.
	 **/
	Viewer( InnerModel *im, QString root="root", osg::Group *parent=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	~Viewer();

public:
	InnerModel* innerModel;
	QHash<QString, Laser*> lasers;
	QHash<QString, Camera*> cameras;
	QHash<QString, RGBD*> rgbds;
	QHash<QString, IMU*> imus;
};



// ------------------------------------------------------------------------------------------------
// Node
// ------------------------------------------------------------------------------------------------

/**
 * @brief Abstracte tree node.
 **/
class Node
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Node name, must be unique.
	 * @param parent_ Node parent, must be an existing node or NULL for the tree root.
	 **/
	Node( QString id_, Node *parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~Node();
	
	/**
	 * @brief Recursively print the node information.
	 *
	 * @param s ...
	 * @param verbose ... Defaults to false.
	 * @return void
	 **/
	void treePrint( QString s, bool verbose=false );
	
	/**
	 * @brief Sets the current
	 *
	 * @param parent_ Pointer to a valid node.
	 * @return void
	 **/
	void setParent(Node *parent_);
	
	/**
	 * @brief Add a new child to the node.
	 *
	 * @param child Pointer to a valid node.
	 * @return void
	 * 
	 * Adds 'child' to the set of children of the node, and sets its parent to the current node.
	 * If 'child' already had a parent, dettaches it from the parent first.
	 **/
	void addChild(Node *child);
	
	/**
	 * @brief ...
	 *
	 * @param f ... Defaults to true.
	 * @return void
	 **/
	void setFixed(bool f=true);
	
	/**
	 * @brief ...
	 *
	 * @return bool
	 **/
	bool isFixed();
	
	/**
	 * @brief Recursively compute the absolute transform from the chain of relative ones.
	 *
	 * @return void
	 **/
	void computeAbsolute();
	
	/**
	 * @brief Recursively compute the chain of relative transforms from the absolute ones.
	 *
	 * @return void
	 **/
	void computeRelative();
	
public:
	const QString id;
	
public:
	Node *im_parent;
	QSet<Node *> im_children;
	RTMat im_pose;
	RTMat im_absolute;
	int im_level;
	bool im_fixed;
	QHash<QString, AttributeType> im_attributes;

protected:
	friend class InnerModel;
	friend class Viewer;
	
	friend class Joint;
	friend class BallJoint;
	friend class GenericJoint;
	friend class HingeJoint;
	friend class PrismaticJoint;
	
	friend class Body;
	friend class DifferentialRobot;
	
	friend class Sensor;
	friend class Camera;
	friend class IMU;
	friend class Laser;
	friend class RGBD;
	
	friend class Primitive;
	friend class Mesh;
	friend class Plane;
	friend class PointCloud;

	virtual void print(bool verbose) = 0;
	virtual void save(QTextStream &out, int tabs) = 0;
	virtual void initOSG( osg::Group* graphicsScene ) = 0;
	virtual void initBullet( btDynamicsWorld* physicsWorld ) = 0;
	virtual void preUpdate( const float elapsed ) = 0;
	virtual void postUpdate( const float elapsed ) = 0;
};



// ------------------------------------------------------------------------------------------------
// Joint
// ------------------------------------------------------------------------------------------------

/**
 * @brief Abstract joint between bodies
 **/
class Joint : public Node
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Joint name, must be unique.
	 * @param jtype_ Joint type, can be Free, Motor or Manual.
	 * @param port_ Joint port, through which the proper ICE interface is exported.
	 * @param parent_ Joint parent, must be an existing node.
	 **/
	Joint( QString id_, JointType jtype_, uint32_t port_=0, Node *parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~Joint();

public:
	// InnerModel attributes
	float im_tx, im_ty, im_tz;
	float im_rx, im_ry, im_rz;
	uint32_t im_port;
	JointType im_jtype;
	
	// Bullet attributes
	btTypedConstraint* bt_constraint;

protected:
	virtual void print(bool verbose) = 0;
	virtual void save(QTextStream &out, int tabs) = 0;
	virtual void initOSG( osg::Group* graphicsScene ) = 0;
	virtual void initBullet( btDynamicsWorld* physicsWorld ) = 0;
	virtual void preUpdate( const float elapsed ) = 0;
	virtual void postUpdate( const float elapsed ) = 0;
};



// ------------------------------------------------------------------------------------------------
// GenericJoint
// ------------------------------------------------------------------------------------------------

/**
 * @brief Joint with full 6 degrees of freedom
 **/
class GenericJoint : public Joint
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Joint name, must be unique.
	 * @param jtype_ Joint type, can be Free, Motor or Manual.
	 * @param tx_ Translation in the X axis, relative to the parent.
	 * @param ty_ Translation in the Y axis, relative to the parent.
	 * @param tz_ Translation in the Z axis, relative to the parent.
	 * @param rx_ Rotation in the X axis, relative to the parent.
	 * @param ry_ Rotation in the Y axis, relative to the parent.
	 * @param rz_ Rotation in the Z axis, relative to the parent.
	 * @param port_ Joint port, through which the proper ICE interface is exported.
	 * @param parent_ Joint parent, must be an existing node.
	 **/
	GenericJoint( QString id_, JointType jtype_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_=0, Node *parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~GenericJoint();
	
	/**
	 * @brief Set new rotation values, relative to the parent.
	 *
	 * @param rx_ Rotation in the X axis.
	 * @param ry_ Rotation in the Y axis.
	 * @param rz_ Rotation in the Z axis.
	 * @return void
	 **/
	void setRotation( const float rx_, const float ry_, const float rz_ );
	
	/**
	 * @brief Set new traslation values, relative to the parent.
	 *
	 * @param tx_ Translation in the X axis.
	 * @param ty_ Translation in the Y axis.
	 * @param tz_ Translation in the Z axis.
	 * @return void
	 **/
	void setTranslation( const float tx_, const float ty_, const float tz_ );
	
	/**
	 * @brief Set new rotation and translation values, relative to the parent.
	 *
	 * @param tx_ Translation in the X axis.
	 * @param ty_ Translation in the Y axis.
	 * @param tz_ Translation in the Z axis.
	 * @param rx_ Rotation in the X axis.
	 * @param ry_ Rotation in the Y axis.
	 * @param rz_ Rotation in the Z axis.
	 * @return void
	 **/
	void setTransform( const float tx_, const float ty_, const float tz_, const float rx_, const float ry_, const float rz_ );

protected:
	virtual void print(bool verbose);
	virtual void save(QTextStream &out, int tabs);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// BallJoint
// ------------------------------------------------------------------------------------------------

/**
 * @brief Joint with 3 degrees of rotational freedom
 **/
class BallJoint : public Joint
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Joint name, must be unique.
	 * @param jtype_ Joint type, can be Free, Motor or Manual.
	 * @param lx_ Lower rotational limit in the X axis.
	 * @param ly_ Lower rotational limit in the Y axis.
	 * @param lz_ Lower rotational limit in the Z axis.
	 * @param hx_ Upper rotational limit in the X axis.
	 * @param hy_ Upper rotational limit in the Y axis.
	 * @param hz_ Upper rotational limit in the Z axis.
	 * @param tx_ Translation in the X axis, relative to the parent.
	 * @param ty_ Translation in the Y axis, relative to the parent.
	 * @param tz_ Translation in the Z axis, relative to the parent.
	 * @param rx_ Rotation in the X axis, relative to the parent.
	 * @param ry_ Rotation in the Y axis, relative to the parent.
	 * @param rz_ Rotation in the Z axis, relative to the parent.
	 * @param port_ Joint port, through which the proper ICE interface is exported.
	 * @param parent_ Joint parent, must be an existing node.
	 **/
	BallJoint( QString id_, JointType jtype_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_=0, Body *parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~BallJoint();
	
	/**
	 * @brief Update the lower rotational limit.
	 *
	 * @param lx_ Lower rotational limit in the X axis.
	 * @param ly_ Lower rotational limit in the Y axis.
	 * @param lz_ Lower rotational limit in the Z axis.
	 * @return void
	 **/
	void setLower( const float lx_, const float ly_, const float lz_ );
	
	/**
	 * @brief Update the upper rotational limit.
	 *
	 * @param hx_ Upper rotational limit in the X axis.
	 * @param hy_ Upper rotational limit in the Y axis.
	 * @param hz_ Upper rotational limit in the Z axis.
	 * @return void
	 **/
	void setUpper( const float hx_, const float hy_, const float hz_ );
	
	/**
	 * @brief Set the current rotation value.
	 *
	 * @param cx_ Rotation in the X axis.
	 * @param cy_ Rotation in the Y axis.
	 * @param cz_ Rotation in the Z axis.
	 * @return void
	 **/
	void setAngle( const float cx_, const float cy_, const float cz_ );
	
	/**
	 * @brief Return the lower rotational limit.
	 *
	 * @param lx_ Lower rotational limit in the X axis.
	 * @param ly_ Lower rotational limit in the Y axis.
	 * @param lz_ Lower rotational limit in the Z axis.
	 * @return void
	 **/
	void getLower( float& lx_, float& ly_, float& lz_ ) const;
	
	/**
	 * @brief Return the upper rotational limit.
	 *
	 * @param hx_ Upper rotational limit in the X axis.
	 * @param hy_ Upper rotational limit in the Y axis.
	 * @param hz_ Upper rotational limit in the Z axis.
	 * @return void
	 **/
	void getUpper( float& hx_, float& hy_, float& hz_ ) const;
	
	/**
	 * @brief Get the current rotation value.
	 *
	 * @param cx_ Rotation in the X axis.
	 * @param cy_ Rotation in the Y axis.
	 * @param cz_ Rotation in the Z axis.
	 * @return void
	 **/
	void getAngle( float& cx_, float& cy_, float& cz_ ) const;

public:
	// InnerModel attributes
	float im_lx, im_ly, im_lz;
	float im_hx, im_hy, im_hz;
	float im_cx, im_cy, im_cz;

protected:
	virtual void print(bool verbose);
	virtual void save(QTextStream &out, int tabs);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// HingeJoint
// ------------------------------------------------------------------------------------------------

/**
 * @brief Joint with 1 degree of rotational freedom
 **/
class HingeJoint : public Joint
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Joint name, must be unique.
	 * @param jtype_ Joint type, can be Free, Motor or Manual.
	 * @param tx_ Translation in the X axis, relative to the parent.
	 * @param ty_ Translation in the Y axis, relative to the parent.
	 * @param tz_ Translation in the Z axis, relative to the parent.
	 * @param rx_ Rotation in the X axis, relative to the parent.
	 * @param ry_ Rotation in the Y axis, relative to the parent.
	 * @param rz_ Rotation in the Z axis, relative to the parent.
	 * @param min_ Lower rotation limit.
	 * @param max_ Upper rotation limit.
	 * @param axis_ Axis of rotation.
	 * @param port_ Joint port, through which the proper ICE interface is exported.
	 * @param parent_ Joint parent, must be an existing node.
	 **/
	HingeJoint(  QString id_, JointType jtype_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_=-INFINITY, float max_=INFINITY, std::string axis_="z", uint32_t port_=0, Body *parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~HingeJoint();
	
	/**
	 * @brief Returns the axis of rotation of the joint.
	 *
	 * @return QVec
	 **/
	QVec unitaryAxis();
	
	/**
	 * @brief Update the lower rotational limit.
	 *
	 * @param min Lower rotational limit.
	 * @return void
	 **/
	void setLower( const float min );
	
	/**
	 * @brief Update the upper rotational limit.
	 *
	 * @param min Upper rotational limit.
	 * @return void
	 **/
	void setUpper( const float min );
	
	/**
	 * @brief Set the current rotation angle.
	 *
	 * @param angle Angle of rotation.
	 * @return float
	 **/
	float setAngle( const float angle );
	
	/**
	 * @brief Return the lower rotational limit.
	 *
	 * @return float
	 **/
	float getLower() const;
	
	/**
	 * @brief Return the upper rotational limit.
	 *
	 * @return float
	 **/
	float getUpper() const;
	
	/**
	 * @brief Return the current angle of rotation.
	 *
	 * @return float
	 **/
	float getAngle() const;

public:
	// InnerModel attributes
	float im_min, im_max, im_angle;
	std::string im_axis;

protected:
	virtual void print(bool verbose);
	virtual void save(QTextStream &out, int tabs);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// PrismaticJoint
// ------------------------------------------------------------------------------------------------

/**
 * @brief Joint with 1 degree of translational freedom
 **/
class PrismaticJoint : public Joint
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Joint name, must be unique.
	 * @param jtype_ Joint type, can be Free, Motor or Manual.
	 * @param tx_ Translation in the X axis, relative to the parent.
	 * @param ty_ Translation in the Y axis, relative to the parent.
	 * @param tz_ Translation in the Z axis, relative to the parent.
	 * @param rx_ Rotation in the X axis, relative to the parent.
	 * @param ry_ Rotation in the Y axis, relative to the parent.
	 * @param rz_ Rotation in the Z axis, relative to the parent.
	 * @param min_ Minimum translation value.
	 * @param max_ Maximum translation value.
	 * @param val_ Initial translation value.
	 * @param offset_ Fixed translation offset.
	 * @param axis_ Axis of movement.
	 * @param port_ Joint port, through which the proper ICE interface is exported.
	 * @param parent_ Joint parent, must be an existing node.
	 **/
	PrismaticJoint(  QString id_, JointType jtype_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_, float max_, float val_, float offset_, std::string axis_="z", uint32_t port_=0, Body *parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~PrismaticJoint();
	
	/**
	 * @brief Update the minimum translation value.
	 *
	 * @param min Minimum translation limit.
	 * @return void
	 **/
	void setLower( const float min );
	
	/**
	 * @brief Update the maximum translation value.
	 *
	 * @param max Maximum translation limit.
	 * @return void
	 **/
	void setUpper( const float max );
	
	/**
	 * @brief Set the current translation.
	 *
	 * @param pos Translation value.
	 * @return float
	 **/
	float setPosition( const float pos );
	
	/**
	 * @brief Returns the minimum translation value.
	 *
	 * @return float
	 **/
	float getLower() const;
	
	/**
	 * @brief Returns the maximum translation value.
	 *
	 * @return float
	 **/
	float getUpper() const;
	
	/**
	 * @brief Returns the current translation.
	 *
	 * @return float
	 **/
	float getPosition() const;

public:
	// InnerModel attributes
	float im_min, im_max, im_offset, im_value;
	std::string im_axis;
	
protected:
	virtual void print(bool verbose);
	virtual void save(QTextStream &out, int tabs);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// Body
// ------------------------------------------------------------------------------------------------

/**
 * @brief A static or dynamic rigid body
 **/
class Body : public Node
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Body name, must be unique.
	 * @param pose_ Initial body pose, translation and rotation.
	 * @param params_ Physical parameters (mass, friction, etc).
	 * @param parent_ Body parent, must be an existing node or NULL for the tree root.
	 **/
	Body( QString id_, const RTMat& pose_, const BodyParams& params_, Node* parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~Body();

protected:
	friend class BallJoint;
	friend class GenericJoint;
	friend class HingeJoint;
	friend class PrismaticJoint;
	friend class Mesh;
	friend class Plane;
	friend class PointCloud;
	
	// Graphic attributes
	osg::MatrixTransform* osg_transform;
	osg::PolygonMode* osg_polygonMode;
	
	// Physics attributes
	BodyParams bt_params;
	btTransform bt_transform;
	btMotionState* bt_motion;
	btRigidBody* bt_body;
	btCollisionShape* bt_shape;
	
protected:
	virtual void print(bool verbose);
	virtual void save(QTextStream &out, int tabs);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// DifferentialRobot
// ------------------------------------------------------------------------------------------------

/**
 * @brief Robot base, a kind of kinematic body
 **/
class DifferentialRobot : public Body
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Body name, must be unique.
	 * @param tx_ Traslation in the X axis.
	 * @param ty_ Traslation in the Y axis.
	 * @param tz_ Traslation in the Z axis.
	 * @param rx_ Rotation in the X axis.
	 * @param ry_ Rotation in the Y axis.
	 * @param rz_ Rotation in the Z axis.
	 * @param port_ Body port, through which the proper ICE interface is exported.
	 * @param parent_ Body parent, must be an existing node or NULL for the tree root.
	 **/
	DifferentialRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_=0, Node *parent_=NULL);
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~DifferentialRobot();
	
	/**
	 * @brief Update the rotation values.
	 *
	 * @param rx_ Rotation in the X axis.
	 * @param ry_ Rotation in the Y axis.
	 * @param rz_ Rotation in the Z axis.
	 * @return void
	 **/
	void setRotation( const float rx_, const float ry_, const float rz_ );
	
	/**
	 * @brief Update the translation values.
	 *
	 * @param tx_ Traslation in the X axis.
	 * @param ty_ Traslation in the Y axis.
	 * @param tz_ Traslation in the Z axis.
	 * @return void
	 **/
	void setTranslation( const float tx_, const float ty_, const float tz_ );
	
	/**
	 * @brief Update the translation and rotation values.
	 *
	 * @param tx_ Traslation in the X axis.
	 * @param ty_ Traslation in the Y axis.
	 * @param tz_ Traslation in the Z axis.
	 * @param rx_ Rotation in the X axis.
	 * @param ry_ Rotation in the Y axis.
	 * @param rz_ Rotation in the Z axis.
	 * @return void
	 **/
	void setTransform( const float tx_, const float ty_, const float tz_, const float rx_, const float ry_, const float rz_ );

public:
	// InnerModel attributes
	uint32_t im_port;
	float im_tx, im_ty, im_tz;
	float im_rx, im_ry, im_rz;
	
protected:
	virtual void print(bool verbose);
	virtual void save(QTextStream &out, int tabs);
};



// ------------------------------------------------------------------------------------------------
// Sensor
// ------------------------------------------------------------------------------------------------

/**
 * @brief Abstract sensor attached to a body
 **/
class Sensor : public Node
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Sensor name, must be unique.
	 * @param port_ Sensor port, through which the proper ICE interface is exported.
	 * @param parent_ Sensor parent, must be an existing body.
	 **/
	Sensor( QString id_, const uint32_t port_, Node *parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~Sensor();
	
public:
	uint32_t im_port;
	
protected:
	virtual void print(bool verbose) = 0;
	virtual void save(QTextStream &out, int tabs) = 0;
	virtual void initOSG( osg::Group* graphicsScene ) = 0;
	virtual void initBullet( btDynamicsWorld* physicsWorld ) = 0;
	virtual void preUpdate( const float elapsed ) = 0;
	virtual void postUpdate( const float elapsed ) = 0;
};



// ------------------------------------------------------------------------------------------------
// Camera
// ------------------------------------------------------------------------------------------------

/**
 * @brief Color video camera
 **/
class Camera : public Sensor
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Sensor name, must be unique.
	 * @param port_ Sensor port, through which the proper ICE interface is exported.
	 * @param width_ Horizontal resolution, in pixels.
	 * @param height_ Vertical resolution, in pixels.
	 * @param focal_ Focal length.
	 * @param ifconfig_ Other configuration parameters.
	 * @param parent_ Sensor parent, must be an existing body.
	 **/
	Camera(QString id_, uint32_t port_, float width_, float height_, float focal_, QString ifconfig_, Node *parent_=NULL);
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~Camera();
	
	/**
	 * @brief Returns the camera parameters.
	 *
	 * @return Parameters as a CameraParams struct.
	 **/
	CameraParams getParams() const;
	
	/**
	 * @brief Returns the current value of the color buffer.
	 *
	 * @return Array of packed RGB values, total array length is 3*width*height.
	 **/
	const uint8_t* getColor() const;
	
	/**
	 * @brief Returns the current value of the depth buffer.
	 *
	 * @return Array of depth values, total array length is width*height.
	 **/
	const float* getDepth() const;

public:
	friend class InnerModel;
	// InnerModel attributes
	Cam im_camera;
	float im_width, im_height, im_focal;
	QString im_ifconfig;
	
	// Graphic attributes
	osg::MatrixTransform* osg_transform;
	osg::Image *osg_rgb;
	osg::Image *osg_depth;
	osgViewer::Viewer *osg_camera;
	osgGA::TrackballManipulator *osg_manipulator;
	
protected:
	virtual void print(bool verbose);
	virtual void save(QTextStream &out, int tabs);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// RGBD
// ------------------------------------------------------------------------------------------------

/**
 * @brief Color plus depth video camera (e.g. a Microsoft Kinect)
 * 
 * Right now, this class works exactly the same as the Camera class, its purpose is to be able to
 * differentiate among different kinds of sensors.
 **/
class RGBD : public Camera
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Sensor name, must be unique.
	 * @param port_ Sensor port, through which the proper ICE interface is exported.
	 * @param width_ Horizontal resolution, in pixels.
	 * @param height_ Vertical resolution, in pixels.
	 * @param focal_ Focal length.
	 * @param ifconfig_ Other configuration parameters.
	 * @param parent_ Sensor parent, must be an existing body.
	 **/
	RGBD( QString id_, uint32_t port_, float width_, float height_, float focal_, QString ifconfig_, Node *parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~RGBD();
	
protected:
	virtual void print(bool verbose);
	virtual void save(QTextStream &out, int tabs);
};



// ------------------------------------------------------------------------------------------------
// IMU
// ------------------------------------------------------------------------------------------------

/**
 * @brief Inertial Measurement Unit
 * 
 * An IMU is a device that measures forces internal to a body, that is, acceleration, torque, etc.
 * Modern IMUs are solid state devices that combine accelerometers, gyroscopes, magnetometers, etc.
 **/
class IMU : public Sensor
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Sensor name, must be unique.
	 * @param port_ Sensor port, through which the proper ICE interface is exported.
	 * @param parent_ Sensor parent, must be an existing node or NULL for the tree root.
	 **/
	IMU(QString id_, uint32_t port_, Node *parent_=NULL);
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~IMU();

protected:
	virtual void save(QTextStream &out, int tabs);
	virtual void print(bool verbose);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// Laser
// ------------------------------------------------------------------------------------------------

/**
 * @brief Laser rangefinder or LIDAR
 **/
class Laser : public Sensor
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Sensor name, must be unique.
	 * @param port_ Sensor port, through which the proper ICE interface is exported.
	 * @param min_ ...
	 * @param max_ ...
	 * @param angle_ ...
	 * @param measures_ ...
	 * @param ifconfig_ ...
	 * @param parent_ Sensor parent, must be an existing node or NULL for the tree root.
	 **/
	Laser(
		QString id_,
		uint32_t port_,
		uint32_t min_,
		uint32_t max_,
		float angle_,
		uint32_t measures_,
		QString ifconfig_,
		Node *parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~Laser();
	
public:
	// InnerModel attributes
	uint32_t im_min, im_max;
	float im_angle;
	uint32_t im_measures;
	QString im_ifconfig;
	
	// Graphic attributes
	osg::MatrixTransform* osg_transform;
	osg::Switch *osg_node;
	
protected:
	virtual void save(QTextStream &out, int tabs);
	virtual void print(bool verbose);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// Primitive
// ------------------------------------------------------------------------------------------------

/**
 * @brief Abstract graphical primitive associated with a body
 **/
class Primitive : public Node
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Primitive name, must be unique.
	 * @param parent_ Primitive parent, must be an existing body.
	 **/
	Primitive( QString id_, Node* parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~Primitive();
    
protected:
	virtual void save(QTextStream &out, int tabs) = 0;
	virtual void print(bool verbose) = 0;
	virtual void initOSG( osg::Group* graphicsScene ) = 0;
	virtual void initBullet( btDynamicsWorld* physicsWorld ) = 0;
	virtual void preUpdate( const float elapsed ) = 0;
	virtual void postUpdate( const float elapsed ) = 0;
};



// ------------------------------------------------------------------------------------------------
// Mesh
// ------------------------------------------------------------------------------------------------

/**
 * @brief Concave triangle mesh
 **/
class Mesh : public Primitive
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Primitive name, must be unique.
	 * @param path_ File path where the mesh is stored, in OSG or IVE format.
	 * @param tx_ Traslation in the X axis, relative to the body center.
	 * @param ty_ Traslation in the Y axis, relative to the body center.
	 * @param tz_ Traslation in the Z axis, relative to the body center.
	 * @param rx_ Rotation in the X axis.
	 * @param ry_ Rotation in the Y axis.
	 * @param rz_ Rotation in the Z axis.
	 * @param sx_ Scale in the X axis.
	 * @param sy_ Scale in the Y axis.
	 * @param sz_ Scale in the Z axis.
	 * @param parent_ Primitive parent, must be an existing body.
	 **/
	Mesh( QString id_,
	      QString path_,
	      float tx_, float ty_, float tz_,
	      float rx_, float ry_, float rz_,
	      float sx_, float sy_, float sz_,
	      Node* parent_=NULL );
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~Mesh();
	
public:
	QString path;
	float tx, ty, tz;
	float rx, ry, rz;
	float sx, sy, sz;
	osg::MatrixTransform* osg_transform;
	
protected:
	virtual void save(QTextStream &out, int tabs);
	virtual void print(bool verbose);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// Plane
// ------------------------------------------------------------------------------------------------

/**
 * @brief Not really a plane, more like an arbitrarily oriented box
 **/
class Plane : public Primitive
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Primitive name, must be unique.
	 * @param texture_ Either a path of the image to use as the texture, or a RGB color in HTML format.
	 * @param width_ Plane size in the X axis.
	 * @param height_ Plane size in the Y axis.
	 * @param depth_ Plane size in the Z axis.
	 * @param repeat_ Should the texture be repeated?
	 * @param nx_ X component of the plane normal.
	 * @param ny_ Y component of the plane normal.
	 * @param nz_ Z component of the plane normal.
	 * @param px_ Plane center in the X axis.
	 * @param py_ Plane center in the Y axis.
	 * @param pz_ Plane center in the Z axis.
	 * @param parent_ Primitive parent, must be an existing body.
	 **/
	Plane(QString id_, QString texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, Node *parent_=NULL);
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~Plane();
	
	/**
	 * @brief Update the plane geometry.
	 *
	 * @param nx_ X component of the plane normal.
	 * @param ny_ Y component of the plane normal.
	 * @param nz_ Z component of the plane normal.
	 * @param px_ Plane center in the X axis.
	 * @param py_ Plane center in the Y axis.
	 * @param pz_ Plane center in the Z axis.
	 * @return void
	 **/
	void updateValues(float nx_, float ny_, float nz_, float px_, float py_, float pz_);
	
	/**
	 * @brief Update the plane texture.
	 *
	 * @param image_ Pointer to the texture buffer, in RGB24 format.
	 * @param width_ Horizontal texture resolution, in pixels.
	 * @param height_ Horizontal texture resolution, in pixels.
	 * @return void
	 **/
	void updateTexture( uint8_t* image_, int32_t width_, int32_t height_ );

public:
	friend class InnerModel;
	
	// InnerModel attributes
	QVec im_normal, im_point;
	QString im_texture;
	float im_width, im_height, im_depth;
	int im_repeat;
	
	// Graphic attributes
	osg::MatrixTransform* osg_transform;
	osg::Geode* osg_geode;
	osg::Texture2D* osg_texture;
	osg::ShapeDrawable* osg_planeDrawable;
	osg::Image* osg_image;
	uint8_t* osg_data;
	int32_t osg_width, osg_height;
	bool osg_dirty;

protected:
	virtual void save(QTextStream &out, int tabs);
	virtual void print(bool verbose);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};



// ------------------------------------------------------------------------------------------------
// PointCloud
// ------------------------------------------------------------------------------------------------

/**
 * @brief Pointcloud
 **/
class PointCloud : public Primitive
{
public:
	/**
	 * @brief Constructor.
	 *
	 * @param id_ Primitive name, must be unique.
	 * @param parent_ Primitive parent, must be an existing body.
	 **/
	PointCloud(QString id_, Node *parent_=NULL);
	
	/**
	 * @brief Destructor.
	 *
	 **/
	virtual ~PointCloud();
	
	/**
	 * @brief ...
	 *
	 * @return void
	 **/
	void reload();
	
	/**
	 * @brief ...
	 *
	 * @return float
	 **/
	float getPointSize() const;
	
	/**
	 * @brief ...
	 *
	 * @param p ...
	 * @return void
	 **/
	void setPointSize( float p );

public:
	// Graphic attributes
	osg::MatrixTransform* osg_transform;
	osg::Geode* osg_geode;
	osg::Vec3Array *osg_points;
	osg::Vec4Array *osg_colors;
	osg::Geometry *osg_geometry;
	osg::TemplateIndexArray <unsigned int, osg::Array::UIntArrayType,4,4> *osg_colorIndexArray;
	osg::DrawArrays *osg_arrays;
	float osg_pointSize;

protected:
	virtual void save(QTextStream &out, int tabs);
	virtual void print(bool verbose);
	virtual void initOSG( osg::Group* graphicsScene );
	virtual void initBullet( btDynamicsWorld* physicsWorld );
	virtual void preUpdate( const float elapsed );
	virtual void postUpdate( const float elapsed );
};

};
