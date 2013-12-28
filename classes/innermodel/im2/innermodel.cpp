/*
 *    Copyright (C) 2010-2013 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "innermodel.h"

// Qt includes
#include <QQueue>
#include <QFileInfo>
#include <QtXml>
#include <QXmlSchema>
#include <QXmlSchemaValidator>

#include <bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

// Robocomp includes
#include <osgviewer/adapterwidget.h>
#include <osgviewer/findnamednode.h>
#include <osgviewer/getworldcoorofnode.h>
#include <osgviewer/viewerqt.h>
#include <QMat/QMatAll>

#include <osg/LightModel>



namespace IM2 {

QString xml_base_path;
const btVector3 GRAVITY( 0.0f, -9.80665f, 0.0f );
void recursiveXML(QDomNode parentDomNode, InnerModel *model, Node *imNode);



// ------------------------------------------------------------------------------------------------
// Free functions
// ------------------------------------------------------------------------------------------------

osg::Vec3 QVecToOSGVec( const QVec &vec )
{
	return osg::Vec3(vec(0), vec(1), -vec(2));
// 	return osg::Vec3(vec(0), vec(2), vec(1));
}



osg::Vec4 htmlStringToOsgVec4( QString color )
{
	QString red   = QString("00");
	QString green = QString("00");
	QString blue  = QString("00");
	bool ok;
	red[0]   = color[1]; red[1]   = color[2];
	green[0] = color[3]; green[1] = color[4];
	blue[0]  = color[5]; blue[1]  = color[6];
	return osg::Vec4(float(red.toInt(&ok, 16))/255., float(green.toInt(&ok, 16))/255., float(blue.toInt(&ok, 16))/255., 0.f);
}



QString osgVec4ToHtmlString( osg::Vec4 color )
{
	QString ret("#");
	QString red   = QString::number(((int)color[0]*255), 16);
	QString green = QString::number(((int)color[1]*255), 16);
	QString blue  = QString::number(((int)color[2]*255), 16);
	return ret + red + green + blue;
}



osg::Matrix QMatToOSGMat4( const RTMat &nodeB )
{
	btTransform temp;
	QMatToBullet( nodeB, temp );
	return osgbCollision::asOsgMatrix( temp );
}


void OsgToQMat( const osg::Matrix& osm, RTMat& mat )
{
	btTransform temp;
	temp = osgbCollision::asBtTransform( osm );
	BulletToQMat( temp, mat );
}



void QMatToBullet( const RTMat& mat, btTransform& trf )
{
	btMatrix3x3& rot = trf.getBasis();
	btVector3& pos = trf.getOrigin();
	
	// X axis
	rot[0][0] = mat(0,0);
	rot[0][1] = mat(0,1);
	rot[0][2] = -mat(0,2);
	pos[0]    = mat(0,3);
	
	// Y axis
	rot[1][0] = mat(1,0);
	rot[1][1] = mat(1,1);
	rot[1][2] = -mat(1,2);
	pos[1]    = mat(1,3);
	
	// Z axis
	rot[2][0] = -mat(2,0);
	rot[2][1] = -mat(2,1);
	rot[2][2] = mat(2,2);
	pos[2]    = -mat(2,3);
}



void BulletToQMat( const btTransform& trf, RTMat& mat )
{
	const btMatrix3x3& rot = trf.getBasis();
	const btVector3& pos = trf.getOrigin();
	
	// X axis
	mat(0,0) = rot[0][0];
	mat(0,1) = rot[0][1];
	mat(0,2) = -rot[0][2];
	mat(0,3) = pos[0];
	
	// Y axis
	mat(1,0) = rot[1][0];
	mat(1,1) = rot[1][1];
	mat(1,2) = -rot[1][2];
	mat(1,3) = pos[1];
	
	// Z axis
	mat(2,0) = -rot[2][0];
	mat(2,1) = -rot[2][1];
	mat(2,2) = rot[2][2];
	mat(2,3) = -pos[2];
	
	// W axis
	mat(3,0) = 0.0f;
	mat(3,1) = 0.0f;
	mat(3,2) = 0.0f;
	mat(3,3) = 1.0;
}



void setMainCamera(osgGA::TrackballManipulator *manipulator, CameraView pov)
{
	osg::Quat mRot;

	switch(pov)
	{
	case TOP_POV:
		mRot.makeRotate(-M_PI_2, QVecToOSGVec(QVec::vec3(1,0,0)));
		break;
	case BACK_POV:
		mRot.makeRotate(M_PI_2,  QVecToOSGVec(QVec::vec3(0,0,0)));
		break;
	case FRONT_POV:
		mRot.makeRotate(M_PI,    QVecToOSGVec(QVec::vec3(0,1,0)));
		break;
	case LEFT_POV:
		mRot.makeRotate(M_PI_2,  QVecToOSGVec(QVec::vec3(0,-1,0)));
		break;
	case RIGHT_POV:
		mRot.makeRotate(M_PI_2,  QVecToOSGVec(QVec::vec3(0,1,0)));
		break;
	default:
		qFatal("Viewer: invalid POV.");
	}

	manipulator->setRotation(mRot);
}



// ------------------------------------------------------------------------------------------------
// Exception
// ------------------------------------------------------------------------------------------------

Exception::Exception(const std::string &reason) : runtime_error(std::string("Exception: ") + reason)
{
	std::cout << reason << std::endl;
}



// ------------------------------------------------------------------------------------------------
// InnerModel
// ------------------------------------------------------------------------------------------------

/// (Con/De)structors
InnerModel::InnerModel()
{
	broadphase = NULL;
	collisionConfiguration = NULL;
	dispatcher = NULL;
	solver = NULL;
	dynamicsWorld = NULL;
	
	// Create the mutex
	mutex = new QMutex(QMutex::Recursive);
	
	// Create the root node
	GenericJoint *root = new GenericJoint( "root", Manual, .0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
	setRoot(root);
	hash["root"] = root;
	
	// How to use:
	//   Transform *tr = innerModel->newTransform("name", parent, rx, ry, rz, px, py, pz);
	//   parent->addChild(tr);
}



InnerModel::InnerModel(std::string xmlFilePath)
{
	broadphase = NULL;
	collisionConfiguration = NULL;
	dispatcher = NULL;
	solver = NULL;
	dynamicsWorld = NULL;
	
	QFileInfo fi( QString::fromStdString( xmlFilePath ) );
	xml_base_path = fi.absolutePath();
	
	// Create the mutex
	mutex = new QMutex(QMutex::Recursive);
	if (not load(QString::fromStdString(xmlFilePath)) )
	{
		qFatal("InnerModelReader::load error using file %s\n", xmlFilePath.c_str());
	}
}



InnerModel::InnerModel(const InnerModel &original)
{
	broadphase = NULL;
	collisionConfiguration = NULL;
	dispatcher = NULL;
	solver = NULL;
	dynamicsWorld = NULL;
}



InnerModel::~InnerModel()
{
	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;
}



bool InnerModel::load(const QString& path)
{
	QByteArray fileContents;
	
	// Open and read the input file
 	qDebug() << "InnerModel: reading" << qPrintable(path);
	QFile file(path);
	if (!file.open(QIODevice::ReadOnly)) {
		qDebug() << "Can't open" << qPrintable(path);
		return false;
	}
	fileContents = file.readAll();
	if( fileContents.size() == 0 ) {
		qDebug() << qPrintable(path) << " is empty or couldn't be read";
		return false;
	}
	file.close();

	// Validate the document
	QXmlSchema schema;
	schema.load( QUrl::fromLocalFile("/opt/robocomp/share/InnerModel2.xsd") );
	QXmlSchemaValidator validator( schema );
	bool isValid = validator.validate( fileContents );
	if( !isValid ) {
		return false;
	}
	
	// Parse the document
	QDomDocument doc("mydocument");
	QString errorMsg;
	int errorLine, errorColumn;
	if (!doc.setContent( fileContents, &errorMsg, &errorLine, &errorColumn )) {
		qDebug() << "Can't set document content from" << qPrintable(path);
		qDebug() << "line:" << errorLine << "  column:" << errorColumn;
		qDebug() << "error:" << errorMsg;
		return false;
	}

	// Build the InnerModel tree
	QDomElement root = doc.documentElement();
	if (root.tagName().toLower() != QString("innerModel").toLower()) {
		qDebug() << "<innerModel> tag missing.";
		return false;
	}
	GenericJoint *r = new GenericJoint( QString("root"), Manual, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
	this->setRoot(r);
//	r->parent = NULL;
	
	recursiveXML(root, this, this->root);

// 	for( int z=0; z<15 ; ++z ) {
// 		for( int x=-5; x<=+5 ; ++x ) {
// 			char name[100];
// 			snprintf( name, 100, "%d%+d\n", z, x );
// 			RTMat pose;
// 			pose.setTr( x*0.2, 0.1+0.2*z, 1.2 );
// 			BodyParams params( Dynamic, 1.0, 0.9, 0.3 );
// 			MeshParams mesh;
// 			mesh.path = "/home/robocomp/loki/models/cube.ive";
// 			mesh.tx = mesh.ty = mesh.tz = 0.0f;
// 			mesh.rx = mesh.ry = mesh.rz = 0.0f;
// 			mesh.sx = mesh.sy = mesh.sz = 0.095f;
// 			Body* b = newBody( name, NULL, pose, params );
// 			b->addMesh( mesh );
// 			b->setParent( this->root );
// 		}
// 	};
	
// 	//treePrint();
	
	return true;
}



bool InnerModel::save(const QString& path)
{
	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return false;
	
	QTextStream out(&file);
	root->save(out, 0);
	file.close();
	return true;
}


void InnerModel::initPhysics( osg::Group* graphicsScene )
{
	// Start the physics engine
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher ( collisionConfiguration );
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld ( dispatcher, broadphase, solver, collisionConfiguration );
	dynamicsWorld->setGravity ( GRAVITY );
	
// 	btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>( dynamicsWorld ->getDispatcher() );
	btGImpactCollisionAlgorithm::registerAlgorithm( dispatcher );
	
	// Initialize the physic attributes of the nodes
	cleanupTables();
	root->computeAbsolute();
	root->initOSG( graphicsScene );
	root->initBullet( dynamicsWorld );
}



void InnerModel::updatePhysics( const float elapsed )
{
	root->computeAbsolute();
	root->preUpdate( elapsed );
	dynamicsWorld->stepSimulation( elapsed, 10, 1.0f/60.0f );
	root->postUpdate( elapsed );
// 	root->computeRelative();
}



///get sub tree and return a list with his id
void InnerModel::getSubTree(Node *node, QStringList& names) const
{
	QSet<Node*>::iterator it;
	for ( it=node->im_children.begin() ; it!=node->im_children.end(); ++it )
	{
		getSubTree(*it, names);
	}
	names.append(node->id);
}



///Remove sub tree and return sa list with his id
void InnerModel::removeSubTree(Node *node)
{
	QSet<Node*>::iterator it;
	for ( it=node->im_children.begin(); it!=node->im_children.end(); ++it )
	{
		removeSubTree(*it);
	}
	node->im_parent->im_children.remove(node);
	removeNode(node->id);
}



InnerModel InnerModel::cloneFake(const QVec & basePose) const
{
	InnerModel rob( *this );
	rob.updateGenericJointValues("base", basePose(0), 0, basePose(1), 0, basePose(2), 0 );
	return rob;
}



void InnerModel::cleanupTables()
{
	QMutexLocker l(mutex);
	localHashTr.clear();
	localHashRot.clear();
}



void InnerModel::updateGenericJointValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId)
{
	cleanupTables();
	
	GenericJoint *aux = dynamic_cast<GenericJoint *>(hash[transformId]);
	if (aux != NULL)
	{
		if (parentId!="")
		{
			GenericJoint *auxParent = dynamic_cast<GenericJoint *>(hash[parentId]);
			if (auxParent!=NULL)
			{
				RTMat Tbi;
				Tbi.setTr( tx,ty,tz);
				Tbi.setR ( rx,ry,rz);
				
				///Tbp Inverse = Tpb. This gets Tpb directly. It's the same
				RTMat Tpb= getTransformationMatrix ( getNode ( transformId)->im_parent->id,parentId );
				///New Tpi
				RTMat Tpi=Tpb*Tbi;
				
				QVec angles =Tpi.extractAnglesR();
				QVec tr=Tpi.getTr();
				
				rx=angles.x();ry=angles.y();rz=angles.z();
				tx=tr.x();ty=tr.y();tz=tr.z();
			}
			else if (hash[parentId] == NULL)
				qDebug() << "There is no such" << parentId << "node";
			else
				qDebug() << "?????";
		}
		//always update
		aux->setTransform( tx, ty, tz, rx, ry, rz );
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz)
{
	cleanupTables();
	
	Plane *plane = dynamic_cast<Plane *>(hash[planeId]);
	if (plane != NULL)
	{
		plane->updateValues(nx, ny, nz, px, py, pz);
	}
	else if (hash[planeId] == NULL)
		qDebug() << "There is no such" << planeId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId)
{
	cleanupTables();
	
	GenericJoint *aux = dynamic_cast<GenericJoint *>(hash[transformId]);
	if (aux != NULL) {
		if (parentId!="") {
			updateGenericJointValues( transformId, tx, ty, tz, 0.0f, 0.0f, 0.0f, parentId );
		}
		else {
			aux->setTranslation( tx, ty, tz );
		}
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::updateRotationValues(QString transformId, float rx, float ry, float rz, QString parentId)
{
	cleanupTables();
	
	GenericJoint *aux = dynamic_cast<GenericJoint *>(hash[transformId]);
	if (aux != NULL)
	{
		if (parentId!="")
		{
			updateGenericJointValues( transformId, 0.0f, 0.0f, 0.0f, rx, ry, rz, parentId );
		}
		else
			aux->setRotation( rx, ry, rz );
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::updateHingeJointValue(QString jointId, float angle)
{
	cleanupTables();
	
	HingeJoint *j = dynamic_cast<HingeJoint *>(hash[jointId]);
	if (j != NULL)
	{
		j->setAngle(angle);
	}
	else if (hash[jointId] == NULL)
		qDebug() << "There is no such" << jointId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::updatePrismaticJointPosition(QString jointId, float pos)
{
	cleanupTables();
	
	PrismaticJoint *j = dynamic_cast<PrismaticJoint *>(hash[jointId]);
	if (j != NULL)
	{
		j->setPosition(pos);
	}
	else if (hash[jointId] == NULL)
		qDebug() << "There is no such" << jointId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::setRoot(Node *node)
{
	root = node;
	hash["root"] = root;
	root->im_parent=NULL;
}



GenericJoint *InnerModel::newGenericJoint(
	QString id,
	Node *parent,
	JointType jtype,
	float tx, float ty, float tz,
	float rx, float ry, float rz,
	uint32_t port )
{
	if (hash.contains(id)) qFatal("InnerModel::newTransform: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	GenericJoint *newnode = new GenericJoint( id, jtype, tx, ty, tz, rx, ry, rz, port, parent );
	hash[id] = newnode;
	return newnode;
}



BallJoint *InnerModel::newBallJoint(
	QString id,
	Body *parent,
	JointType jtype,
	float lx, float ly, float lz,
	float hx, float hy, float hz,
	float tx, float ty, float tz,
	float rx, float ry, float rz,
	uint32_t port )
{
	if (hash.contains(id)) qFatal("InnerModel::newJoint: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	BallJoint *newnode = new BallJoint( id, jtype, lx, ly, lz, hx, hy, hz, tx, ty, tz, rx, ry, rz, port, parent);
	hash[id] = newnode;
	return newnode;
}



HingeJoint *InnerModel::newHingeJoint(
	QString id,
	Body* parent,
	JointType jtype,
	float tx, float ty, float tz,
	float rx, float ry, float rz,
	float min, float max,
	std::string axis,
	uint32_t port )
{
	if (hash.contains(id)) qFatal("InnerModel::newJoint: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	HingeJoint *newnode = new HingeJoint( id, jtype, tx, ty, tz, rx, ry, rz, min, max, axis, port, parent );
	hash[id] = newnode;
	return newnode;
}



PrismaticJoint *InnerModel::newPrismaticJoint(
	QString id,
	Body* parent,
	JointType jtype,
	float tx, float ty, float tz,
	float rx, float ry, float rz,
	float min, float max, float value, float offset,
	std::string axis,
	uint32_t port )
{
	if (hash.contains(id)) qFatal("InnerModel::newPrismaticJoint: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	PrismaticJoint *newnode = new PrismaticJoint( id, jtype, tx, ty, tz, rx, ry, rz, min, max, value, offset, axis, port, parent);
	hash[id] = newnode;
	return newnode;
}



DifferentialRobot *InnerModel::newDifferentialRobot(QString id, Node *parent, float tx, float ty, float tz, float rx, float ry, float rz, uint32_t port)
{
	if (hash.contains(id)) qFatal("InnerModel::newDifferentialrobot: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	DifferentialRobot *newnode = new DifferentialRobot(id, tx, ty, tz, rx, ry, rz, port, parent);
	hash[id] = newnode;
	return newnode;
}



Camera *InnerModel::newCamera(QString id, Node *parent, float width, float height, float focal, uint32_t port, QString ifconfig)
{
	if (hash.contains(id)) qFatal("InnerModel::newCamera: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	Camera *newnode = new Camera(id, port, width, height, focal, ifconfig, parent);
	hash[id] = newnode;
	return newnode;
}



RGBD *InnerModel::newRGBD(QString id, Node *parent, float width, float height, float focal, uint32_t port, QString ifconfig)
{
	if (hash.contains(id)) qFatal("InnerModel::newRGBD: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	RGBD *newnode = new RGBD(id, port, width, height, focal, ifconfig, parent);
	hash[id] = newnode;
	return newnode;
}



IMU *InnerModel::newIMU(QString id, Node *parent, uint32_t port)
{
	if (hash.contains(id)) qFatal("InnerModel::newIMU: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	IMU *newnode = new IMU(id, port, parent);
	hash[id] = newnode;
	// 	printf("newIMU id=%s  parentId=%s port=%d\n", id.toStdString().c_str(), parent->id.toStdString().c_str(), port);
	return newnode;
}



Laser *InnerModel::newLaser(QString id, Node *parent, uint32_t port, uint32_t min, uint32_t max, float angle, uint32_t measures, QString ifconfig)
{
	if (hash.contains(id)) qFatal("InnerModel::newLaser: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	// 	printf("newLaser id=%s  parentId=%s port=%d min=%d max=%d angle=%f measures=%d\n", id.toStdString().c_str(), parent->id.toStdString().c_str(), port, min, max, angle, measures);
	Laser *newnode = new Laser(id, port, min, max, angle, measures, ifconfig, parent);
	hash[id] = newnode;
	return newnode;
}



Plane *InnerModel::newPlane(QString id, Node *parent, QString texture, float width, float height, float depth, int repeat, float nx, float ny, float nz, float px, float py, float pz)
{
	if (hash.contains(id)) qFatal("InnerModel::newPlane: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	Plane *newnode = new Plane(id, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz, parent);
	hash[id] = newnode;
	return newnode;
}



PointCloud *InnerModel::newPointCloud(QString id, Node *parent)
{
	if (hash.contains(id)) qFatal("InnerModel::newPointCloud: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	PointCloud *newnode = new PointCloud(id, parent);
	hash[id] = newnode;
// 	printf("Inserted point cloud %s ptr(%p), on node %s\n", id.toStdString().c_str(), newnode, parent->id.toStdString().c_str());
	return newnode;
}



Body* InnerModel::newBody( QString id, Node* parent, const RTMat& pose, const BodyParams& params )
{
	if (hash.contains(id)) qFatal("InnerModel::newBody: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	Body* newnode = new Body(id, pose, params, parent );
	hash[id] = newnode;
// 	printf( "Inserted rigid body %s ptr(%p), on node %s\n", id.toStdString().c_str(), newnode, parent->id.toStdString().c_str());
	return newnode;
}



GenericJoint *InnerModel::getGenericJoint( const QString& id ) const
{
	GenericJoint *tr = dynamic_cast<GenericJoint *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such transform %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a transform", id.toStdString().c_str());
	}
	return tr;
}



BallJoint *InnerModel::getBallJoint( const QString& id ) const
{
	BallJoint *tr = dynamic_cast<BallJoint *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such joint %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a joint", id.toStdString().c_str());
	}
	return tr;
}



HingeJoint *InnerModel::getHingeJoint( const QString& id ) const
{
	HingeJoint *tr = dynamic_cast<HingeJoint *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such joint %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a joint", id.toStdString().c_str());
	}
	return tr;
}



PrismaticJoint *InnerModel::getPrismaticJoint( const QString& id ) const
{
	PrismaticJoint *tr = dynamic_cast<PrismaticJoint *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such joint %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a prismatic joint", id.toStdString().c_str());
	}
	return tr;
}



Body* InnerModel::getBody( const QString& id ) const
{
	Body *body = dynamic_cast<Body *>(hash[id]);
	if (not body)
	{
		if (not hash[id])
			qFatal("No such rigid body %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a rigid body", id.toStdString().c_str());
	}
	return body;
}



DifferentialRobot *InnerModel::getDifferentialRobot( const QString& id ) const
{
	DifferentialRobot *diff = dynamic_cast<DifferentialRobot *>(hash[id]);
	if (not diff)
	{
		if (not hash[id])
			qFatal("No such differential robot %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a differential robot", id.toStdString().c_str());
	}
	return diff;
}



Camera *InnerModel::getCamera( const QString& id ) const
{
	Camera *camera = dynamic_cast<Camera *>(hash[id]);
	if (not camera)
	{
		if (not hash[id])
			qFatal("No such camera %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a camera", id.toStdString().c_str());
	}
	return camera;
}



RGBD *InnerModel::getRGBD( const QString& id ) const
{
	RGBD *camera = dynamic_cast<RGBD *>(hash[id]);
	if (not camera)
	{
		if (not hash[id])
			qFatal("No such camera %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a camera", id.toStdString().c_str());
	}
	return camera;
}



IMU *InnerModel::getIMU( const QString& id ) const
{
	IMU *imu = dynamic_cast<IMU *>(hash[id]);
	if (not imu)
	{
		if (not hash[id])
			qFatal("No such innertial unit %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be an innertial unit", id.toStdString().c_str());
	}
	return imu;
}



Laser *InnerModel::getLaser( const QString& id ) const
{
	Laser *laser = dynamic_cast<Laser *>(hash[id]);
	if (not laser)
	{
		if (not hash[id])
			qFatal("No such laser %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be an laser", id.toStdString().c_str());
	}
	return laser;
}



Mesh *InnerModel::getMesh( const QString& id ) const
{
	Mesh *mesh = dynamic_cast<Mesh *>(hash[id]);
	if (not mesh)
	{
		if (not hash[id])
			qFatal("No such mesh %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a mesh", id.toStdString().c_str());
	}
	return mesh;
}



Plane *InnerModel::getPlane( const QString& id ) const
{
	Plane *plane = dynamic_cast<Plane *>(hash[id]);
	if (not plane)
	{
		if (not hash[id])
			qFatal("No such plane %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a plane", id.toStdString().c_str());
	}
	return plane;
}



PointCloud *InnerModel::getPointCloud( const QString& id ) const
{
	PointCloud *pointcloud = dynamic_cast<PointCloud *>(hash[id]);
	if (not pointcloud)
	{
		if (not hash[id])
			qFatal("No such pointcloud %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a pointcloud", id.toStdString().c_str());
	}
	return pointcloud;
}



/// Stereo geometry

/**
 * \brief Computes essential and fundamental matrices for the pair of cameras stated in the parameters
 * @param firstCam id of the firt camera
 * @param secondCam id of the second camera
 */
// TODO COMPROBAR
void InnerModel::updateStereoGeometry(const QString &firstCam, const QString &secondCam)
{
	this->essential.set(this->getRotationMatrixTo(secondCam, firstCam), this->getTranslationVectorTo(secondCam, firstCam) );
	this->fundamental.set(
		essential,
		dynamic_cast<Camera *>(hash[firstCam])->im_camera,
		dynamic_cast<Camera *>(hash[secondCam])->im_camera );
}



/**
 * \brief Computes de 3D triangulation of two correspondent image points in robot reference system
 * @param left 2D image reference system point
 * @param right 2D image reference system point
 * @return 3D point in robot(base) reference system
 */
// TODO COMPROBAR
QVec InnerModel::compute3DPointInCentral(const QString & firstCam, const QVec & first, const QString & secondCam, const QVec & second)
{
	T detA, a/*, b*/, c;
	
	QVec pI = this->getRotationMatrixTo("central", "firstCam") * static_cast<Camera *>(hash[firstCam])->im_camera.getRayHomogeneous( first );
	QVec pD = this->getRotationMatrixTo("central", "secondCam") * static_cast<Camera *>(hash[secondCam])->im_camera.getRayHomogeneous( second );
	// 	QVec pI = (centralToLeftMotor.getR().transpose() * leftMotorToLeftCamera.getR().transpose()) * leftCamera.getRayHomogeneous( left );
	// 	QVec pD = (centralToRightMotor.getR().transpose() * rightMotorToRightCamera.getR().transpose()) * rightCamera.getRayHomogeneous( right );
	
	QVec n = QVec::vec3( pI(1)-pD(1) , -pI(0)+pD(0) , pI(0)*pD(1)-pD(0)*pI(1) );
	
	QMat A(3,3);
	A(0,0)=pI(0);  A(0,1)=-pD(0);  A(0,2)=n(0);
	A(1,0)=pI(1);  A(1,1)=-pD(1);  A(1,2)=n(1);
	A(2,0)=1;      A(2,1)=-1;      A(2,2)=n(2);
	
	detA = A(0,0)*(A(1,1)*A(2,2)-A(1,2)*A(2,1))-A(0,1)*(A(1,0)*A(2,2)-A(1,2)*A(2,0))+A(0,2)*(A(1,0)*A(2,1)-A(1,1)*A(2,0));
	
	float baseLine = this->getBaseLine(); //NOT IMPLEMENTED
	a = baseLine*(-pD(1)*n(2)+n(1))/detA;
	// 	b = baseLine*(pI(1)*n(2)-n(1))/detA;
	c = baseLine*(-pI(1)+pD(1))/detA;
	
	QVec res(3);
	res(0) = (a*pI(0)-(baseLine/2.))+(c*n(0))/2.;
	res(1) = a*pI(1) + c*n(1)/2.;
	res(2) = a*pI(2) + c*n(2)/2.;
	
	return res;
}



//
// TODO COMPROBAR
QVec InnerModel::compute3DPointInRobot(const QString & firstCamera, const QVec & left, const QString & secondCamera, const QVec & right)
{
	return transform("central", this->compute3DPointInCentral( firstCamera, left, secondCamera, right), "base");
}



QVec InnerModel::compute3DPointFromImageCoords(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem)
{
	QVec pI(3), pD(3), n(3), ray(3), T(3), TI(3), TD(3), pR(0), abc(3);
	QMat A(3,3);
	
	ray = backProject(firstCamera, left);
	pI = getRotationMatrixTo(refSystem, firstCamera)*ray;
	
	
	pI(0)=pI(0)/pI(2);
	pI(1)=pI(1)/pI(2);
	pI(2)=1.;
	
	ray = backProject(secondCamera, right);
	pD = getRotationMatrixTo(refSystem, secondCamera)*ray;
	
	pD(0)=pD(0)/pD(2);
	pD(1)=pD(1)/pD(2);
	pD(2)=1.;
	
	n = pI ^ pD;
	
	A(0,0)=pI(0);  A(0,1)=-pD(0);  A(0,2)=n(0);
	A(1,0)=pI(1);  A(1,1)=-pD(1);  A(1,2)=n(1);
	A(2,0)=pI(2);  A(2,1)=-pD(2);  A(2,2)=n(2);
	
	TI = getTranslationVectorTo(refSystem, firstCamera).fromHomogeneousCoordinates();
	TD = getTranslationVectorTo(refSystem, secondCamera).fromHomogeneousCoordinates();
	T = TD - TI ;
	
	abc = (A.invert())*T;
	
	pR = (pI*abc(0));
	pR = pR + TI;
	pR = (n*(abc(2)/2)) + pR;
	
	return pR;
	
}



QVec InnerModel::compute3DPointFromImageAngles(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem)
{
	QVec pI(3), pD(3), n(3), ray(3), T(3), TI(3), TD(3), pR(0), abc(3);
	QMat A(3,3);
	
	ray(0) = tan(left(0));
	ray(1) = tan(left(1));
	ray(2) = 1.;
	pI = ray;//getRotationMatrixTo(refSystem, firstCamera)*ray;
	
	
	pI(0)=pI(0)/pI(2);
	pI(1)=pI(1)/pI(2);
	pI(2)=1.;
	
	ray(0) = tan(right(0));
	ray(1) = tan(right(1));
	ray(2) = 1.;
	pD = ray;//getRotationMatrixTo(refSystem, secondCamera)*ray;
	
	pD(0)=pD(0)/pD(2);
	pD(1)=pD(1)/pD(2);
	pD(2)=1.;
	
	
	n = pI ^ pD;
	
	A(0,0)=pI(0);  A(0,1)=-pD(0);  A(0,2)=n(0);
	A(1,0)=pI(1);  A(1,1)=-pD(1);  A(1,2)=n(1);
	A(2,0)=pI(2);  A(2,1)=-pD(2);  A(2,2)=n(2);
	
	TI = getTranslationVectorTo(refSystem, firstCamera).fromHomogeneousCoordinates();
	TD = getTranslationVectorTo(refSystem, secondCamera).fromHomogeneousCoordinates();
	T = TD - TI;
	
	abc = (A.invert())*T;
	
	pR = (pI*abc(0));
	pR = pR + TI;
	pR = (n*(abc(2)/2)) + pR;
	
	return pR;
	
}



/// Information retrieval methods
QVec InnerModel::transform(const QString &destId, const QVec &initVec, const QString &origId)
{	
	return (getTransformationMatrix(destId, origId) * initVec.toHomogeneousCoordinates()).fromHomogeneousCoordinates();
}

QVec InnerModel::project(QString reference, QVec origVec, QString cameraId)
{
	origVec = transform(cameraId, origVec, reference);
	
	QVec pc;
	Camera *camera=NULL;
	
	camera = dynamic_cast<Camera *>(hash[cameraId]);
	if (not camera)
		qFatal("No such %s camera", qPrintable(cameraId));
	
	pc = camera->im_camera.project(origVec);
	
	return QVec::vec3(pc(0), pc(1), origVec.norm2());
}



/**
 * \brief Retro-projection function, defines a line in the camera reference system which can be parametrized by the depth s with the expression:
 * p = s*[ (u-u0) / alfaU ; (v-v0) / alfaV ; 1] being alfaU and alfaV the horizontal and vertical focals of the camera (in pixels)
 * p has value 1 in the z axis, the one going out the camera.
 * @param cameraId name of camara to be used as known in innermodel tree
 * @param coord  point in image coordinates
 * @return a line en camera reference system that can be parametrized by the depth s
 */
QVec InnerModel::backProject( const QString &cameraId, const QVec &	coord) //const
{
	if(hash.contains(cameraId))
	{
		QVec p = static_cast<Camera *>(hash[cameraId])->im_camera.getRayHomogeneous(coord);
		return p;
	}
	return QVec();
}



void InnerModel::imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS)
{
	QVec ray = backProject(cameraId, coord);
	
	QVec finalRay = getRotationMatrixTo(anglesRefS, cameraId)*ray;
	
	pan = atan2(finalRay(0), finalRay(2));
	tilt = atan2(finalRay(1), finalRay(2));
	
}



QVec InnerModel::anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS)
{
	QVec p(3), ray(3);
	
	p(0) = tan(pan);
	p(1) = tan(tilt);
	p(2) = 1;
	
	ray = getRotationMatrixTo(cameraId, anglesRefS) * p;
	ray(0)=ray(0)/ray(2);
	ray(1)=ray(1)/ray(2);
	ray(2)=1;
	
	return project(cameraId, ray, cameraId);
	
}



QVec InnerModel::imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString reference)
{
	//We obtain a 3D line (a,b,1) in camera reference system that can be parametrized in depth to obtain a point at "depth" from the camera.
	QVec p = backProject( cameraId, coord ) * depth;
	//Now we transform it to requested node of the robot.
	if(p.size()>0)
		return transform(reference, p, cameraId);
	return p;
}



QVec InnerModel::projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist)
{
	QMat mSystem(3,3);
	QVec tIndep(3);
	QVec pCam(3);
	QVec res(3);
	float dxz, dyz;
	
	pCam(0) = -coord(0)+getCameraWidth(cameraId)/2;
	pCam(1) = -(coord(1)-getCameraHeight(cameraId)/2);
	pCam(2) = getCameraFocal(cameraId);
	QVec pDest = transform(to, pCam, cameraId);
	QVec pCent = transform(to, QVec::vec3(0,0,0), cameraId);
	QVec direc = pDest-pCent;
	dxz = direc(0)/direc(2);
	dyz = direc(1)/direc(2);
	
	res(2) = dist + vPlane(0)*(dxz*pCent(2)-pCent(0)) + vPlane(1)*(dyz*pCent(2)-pCent(1));
	res(2) = res(2)/(vPlane(0)*dxz+vPlane(1)*dyz+vPlane(2));
	res(0)=dxz*(res(2)-pCent(2))+pCent(0);
	res(1)=dyz*(res(2)-pCent(2))+pCent(1);
	
	/*	res.print("res");
	 * 
	 *	mSystem(0,0) = vPlane(0);         mSystem(0,1) = vPlane(1);         mSystem(0,2) = vPlane(2);
	 *	mSystem(1,0) = 0;                 mSystem(1,1) = pCent(2)-pDest(2); mSystem(1,2) = pDest(1)-pCent(1);
	 *	mSystem(2,0) = pDest(2)-pCent(2); mSystem(2,1) = 0;                 mSystem(2,2) = pCent(0)-pDest(0);
	 *	tIndep(0) = dist;
	 *	tIndep(1) = pCent(2)*(pDest(1)-pCent(1))+pCent(1)*(pCent(2)-pDest(2));
	 *	tIndep(2) = pCent(0)*(pDest(2)-pCent(2))+pCent(2)*(pCent(0)-pDest(0));
	 * 
	 * 	return (mSystem.invert())*tIndep;*/
	return res;
}



//
// bool InnerModel::check3DPointInsideFrustrum(QString cameraId, QVec coor)
// {
// }

/**
 * \brief Returns a 3D vector (A,B,C) containing the horizon line for the specified camera+plane in the form 'Ax + By + C = 0'.
 *
 * <p>
 * Returns a 3D vector (A,B,C) containing the horizon line in the form Ax + By + C = 0. For general lines, it will also work as 'y = Ax + C' (not for vertical lines, which are a very rare case).
 * You can check B to know if the returned vector is a regular line:
 * </p>
 * <p>
 * QVec horizon = innerModel->horizonLine("floor", "mycamera", );
 * <br>
 * if (horizon(1) == 0) printf("Vertical horizon.\n");
 * <br>
 * else printf("Regular horizon.\n");
 * </p>
 */
QVec InnerModel::horizonLine(QString planeId, QString cameraId, float heightOffset)
{
	QMutexLocker l(mutex);
	// 	printf("-------------------------------------- cam:%s plane:%s\n", qPrintable(cameraId), qPrintable(planeId));
	// Get camera and plane pointers
	Plane *plane = getPlane(planeId);
	Camera *camera = getCamera(cameraId);
	// Transform rotate plane normal vector to camera reference system
	QMat rtm = getRotationMatrixTo(cameraId, planeId);
	QVec vec = QVec::vec3(plane->im_normal(0), plane->im_normal(1), plane->im_normal(2));
	QVec normal = rtm*vec;
	if (normal(1) <= 0.0000002) throw false;
	
	// Create two points
	QVec p1=QVec::vec3(0., 0., 0.), p2=QVec::vec3(0., 0., 0.);
	// Move both points forward
	p1(2) = p2(2) =  1000.;
	if (normal(1) > 0.0000001) p1(1) = p2(1) = p1(2)*normal(2)/normal(1);
	// Move points left/right-wards
	if (normal(1) > 0.0000001) p1(1) -=  200.*normal(0)/normal(1);
	p1(0) =  200.;
	if (normal(1) > 0.0000001) p2(1) -= -200.*normal(0)/normal(1);
	p2(0) = -200.;
	// Project points
	p1 = project(cameraId, p1, cameraId);
	p2 = project(cameraId, p2, cameraId);
	// Compute image line
	double dx=p2(0)-p1(0);
	double dy=p2(1)-p1(1);
	
	if (abs(dx) <= 1) {
		if (abs(dy) <= 1) {
			throw "Degenerated camera";
		}
		else {
			return QVec::vec3(-1, 0, p1(0));
		}
	}
	else {
		return QVec::vec3(dy/dx, -1, camera->im_camera.getHeight()-(p1(1)-(dy*p1(0)/dx))+heightOffset);
	}
}



/// Matrix transformation retrieval methods
RTMat InnerModel::getTransformationMatrix(const QString &to, const QString &from)
{
	RTMat ret;
	
	QMutexLocker l(mutex);
	if (localHashTr.contains(QPair<QString, QString>(to, from)))
	{
		ret = localHashTr[QPair<QString, QString>(to, from)];
	}
	else
	{
		setLists(from, to);
		foreach (Node *i, listA)
		{
			ret = i->im_pose * ret;
			//ret = ((RTMat)(*i)).operator*(ret);
		}
		foreach (Node *i, listB)
		{
			ret = i->im_pose.invert() * ret;
		}
		localHashTr[QPair<QString, QString>(to, from)] = ret;
	}
	return RTMat(ret);
}



QMat InnerModel::getRotationMatrixTo(const QString &to, const QString &from)
{
	QMat rret = QMat::identity(3);
	
	QMutexLocker l(mutex);
	
	if (localHashRot.contains(QPair<QString, QString>(to, from)))
	{
		rret = localHashRot[QPair<QString, QString>(to, from)];
	}
	else
	{
		setLists(from, to);
		Joint *tf=NULL;
		
		foreach (Node *i, listA)
		{
			if ((tf=dynamic_cast<Joint *>(i))!=NULL)
			{
				rret = tf->im_pose.getR() * rret;
			}
		}
		foreach (Node *i, listB)
		{
			if ((tf=dynamic_cast<Joint *>(i))!=NULL)
			{
				rret = tf->im_pose.getR().transpose() * rret;
			}
		}
		localHashRot[QPair<QString, QString>(to, from)] = rret;
	}
	
	return rret;
}



QVec InnerModel::getTranslationVectorTo(const QString &to, const QString &from)
{
	QMat m = this->getTransformationMatrix(to, from);
	return m.getCol(3);
}



QMat InnerModel::getHomographyMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	QVec planeN = getPlane(plane)->im_normal;
	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = transform(sourceCamera, getPlane(plane)->im_point, plane);
	
	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = getCamera(sourceCamera)->im_camera;
	QMat K2 = getCamera(virtualCamera)->im_camera;
	
	double d = -(planePoint*planeN);
	QMat H = K2 * ( R - ((t*n.transpose()) / d) ) * K1.invert();
	return H;
}



QMat InnerModel::getAffineHomographyMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	QVec planeN = getPlane(plane)->im_normal;
	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = transform(sourceCamera, getPlane(plane)->im_point, plane);
	
	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = getCamera(sourceCamera)->im_camera;
	
	double d = -(planePoint*planeN);
	QMat H = ( R - ((t*n.transpose()) / d) ) * K1.invert();
	for (int r=0;r<2;r++)
		for (int c=0;c<3;c++)
			H(r,c) = H(r,c) * 1000.;
		return H;
}



QMat InnerModel::getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	QVec planeN = getPlane(plane)->im_normal;
	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = transform(sourceCamera, getPlane(plane)->im_point, plane);
	
	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = getCamera(sourceCamera)->im_camera;
	
	double d = -(planePoint*planeN);
	QMat H = ( R - ((t*n.transpose()) / d) ) * K1.invert();
	QMat HFinal(4,3);
	HFinal.inject(H, 0, 0);
	HFinal = HFinal*(1000*1000);
	HFinal(3,0)=1000*H(2,0);
	HFinal(3,1)=1000*H(2,1);
	HFinal(3,2)=1000*H(2,2);
	return HFinal;
}



void InnerModel::setLists(const QString & origId, const QString & destId)
{
	Node *a=hash[origId], *b=hash[destId];
	if (!a)
		throw Exception("Cannot find node: \""+ origId.toStdString()+"\"");
	if (!b)
		throw Exception("Cannot find node: "+ destId.toStdString()+"\"");
	
	int minLevel = (a->im_level < b->im_level) ? (a->im_level) : (b->im_level);
	listA.clear();
	while (a->im_level >= minLevel)
	{
		listA.push_back(a);
		if(a->im_parent == NULL)
		{
			// 			qFatal("InnerModel::setLists: It wouldn't be here!!!!");
			break;
		}
		a=a->im_parent;
	}
	
	listB.clear();
	while (b->im_level >= minLevel)
	{
		listB.push_front(b);
		if(b->im_parent == NULL)
		{
			// 			qFatal("InnerModel::setLists: It wouldn't be here!!!!");
			break;
		}
		b=b->im_parent;
	}
	while (b!=a)
	{
		listA.push_back(a);
		listB.push_front(b);
		a = a->im_parent;
		b = b->im_parent;
	}
}



/// Robex Base specific getters
float InnerModel::getCameraFocal(const QString & cameraId) const
{
// 	return static_cast<Camera *>(getNode(cameraId))->im_camera.getFocal();
	CameraParams p = getCamera( cameraId )->getParams();
	return p.focal;
}



int InnerModel::getCameraWidth(QString cameraId) const
{
	//return getCamera(cameraId)->im_camera.getWidth();
	CameraParams p = getCamera( cameraId )->getParams();
	return p.width;
}



int InnerModel::getCameraHeight(const QString & cameraId) const
{
// 	return static_cast<Camera *>(getNode(cameraId))->im_camera.getHeight();
	CameraParams p = getCamera( cameraId )->getParams();
	return p.height;
}



int InnerModel::getCameraSize(const QString & cameraId) const
{
	//return static_cast<Camera *>(getNode(cameraId))->im_camera.getSize();
	CameraParams p = getCamera( cameraId )->getParams();
	return p.width * p.height;
}



/**
 * \brief Returns current copy of fundamenta matrix as a float h[3][3] array
 * @param h[][] [3][3] preallocates array of floats to contain de fundamental matrix
 */
void InnerModel::getFundamental(float h[3][3]) const
{
	h[0][0] = fundamental(0,0);
	h[0][1] = fundamental(0,1);
	h[0][2] = fundamental(0,2);
	h[1][0] = fundamental(1,0);
	h[1][1] = fundamental(1,1);
	h[1][2] = fundamental(1,2);
	h[2][0] = fundamental(2,0);
	h[2][1] = fundamental(2,1);
	h[2][2] = fundamental(2,2);
}



QMat InnerModel::getFundamental() const
{
	return fundamental;
}



//
QVec InnerModel::robotToWorld(const QVec & vec)
{
	return transform("world", vec, "base");
}



//
QVec InnerModel::robotInWorld()
{
	return transform("world", QVec::vec3(0,0,0), "base");
}



//
float InnerModel::getBaseX()
{
	return transform("world", QVec::vec3(0,0,0), "base")(0);
}



//
float InnerModel::getBaseZ()
{
	return transform("world", QVec::vec3(0,0,0), "base")(2);
}



//
float InnerModel::getBaseAngle()
{
	return getRotationMatrixTo("world", "base").extractAnglesR()(1);
}



float InnerModel::getBaseRadius() //OJO Get from XML file
{
	return 200.f;
}



//
QVec InnerModel::getBaseOdometry()
{
	QVec res = transform("world", QVec::vec3(0,0,0), "base");
	res(1) = res(2);
	res(2) = getRotationMatrixTo("world", "base").extractAnglesR()(1);
	return res;
}



////////////////////////////////////////////
// LASER
////////////////////////////////////////////
/**
 * \brief Local laser measure with index i in laser array is converted to Any RS
 * @param i indexof laser array
 * @return 3Dpoint in World RS
 */
QVec InnerModel::laserTo(const QString &dest, const QString & laserId, const QVec &p)
{
	return transform(dest, p, laserId);
}



/**
 * \brief Local laser measure of range r and angle alfa is converted to Any RS
 * @param r range measure
 * @param alfa angle measure
 * @return 3-vector of x,y,z coordinates un WRS
 */
QVec InnerModel::laserTo(const QString &dest, const QString & laserId , float r, float alpha)
{
	QVec p(3);
	p(0) = r * sin(alpha);
	p(1) = 0 ;
	p(2) = r * cos(alpha);
	return transform(dest, p , laserId);
}



/**
 * \brief Converts a 3D point en WRS to Laser reference system
 * @param world 3D coordinates of a world point as a 3-vector
 * @return 3D coordinates of given point seen from Laser reference system
 */
//
QVec InnerModel::worldToLaser(const QString & laserId, const QVec & p)
{
	return transform(laserId, p, "world");
}



/**
 * \brief Converts a 3D point in Laser (range,angle) coordinates to Robot reference system
 * @param r range measure
 * @param alfa angle measure
 * @return 3-vector of 3D point in Robot reference system
 */
//
QVec InnerModel::laserToBase(const QString & laserId, float r, float alpha)
{
	QVec p(3);
	p(0) = r * sin ( alpha ) ;
	p(2) = r * cos ( alpha ) ;
	p(1) = 0 ; // Laser reference system
	return transform("base", p , laserId);
}



void InnerModel::print(QString s)
{
	treePrint(s, true);
}



void InnerModel::treePrint(QString s, bool verbose)
{
	root->treePrint(QString(s), verbose);
}



QVec InnerModel::laserToWorld( const QString &laserId , const QVec &p)
{
	return laserTo("world", laserId, p);
}



QVec InnerModel::laserToWorld( const QString &laserId , float r, float alfa)
{
	return laserTo("world", laserId, r, alfa);
}



QVec InnerModel::laserToRefFrame(const QString & laserId , float r, float alpha, const QString & refFrame)
{
	return laserTo(refFrame, laserId, r, alpha);
}



QList<QString> InnerModel::getIDKeys()
{
	return hash.keys();
}



Node *InnerModel::getNode(const QString & id) const
{
	if (hash.contains(id)) return hash[id];
	else return NULL;
}



void InnerModel::removeNode(const QString & id) 
{
	hash.remove(id);
}



int InnerModel::debugLevel(int level)
{
	static int debug_level=0; if (level>-1) debug_level=level; return debug_level;
}



Node *InnerModel::getRoot()
{
	return root;
}



void recursiveXML(QDomNode parentDomNode, InnerModel *model, Node *imNode)
{
	QMap<QString, QStringList> nodeAttributes;
	
	Node *node;
	QDomElement e;
	for (QDomNode domNode = parentDomNode.firstChild(); not domNode.isNull(); domNode = domNode.nextSibling()) {
		// Try to create the node
		e = domNode.toElement();
		if (e.isNull()) {
			continue;
		}
		else if (e.tagName().toLower() == "innermodel") {
			qFatal("Tag <innerModel> can only be the root tag.");
			return;
		}
		// ------------------------------------------------------------
		// Joints start here
		// ------------------------------------------------------------
		else if (e.tagName().toLower() == "generic-joint") {
// 			Body * im = dynamic_cast<Body *>(imNode );
			
			JointType jtype;
			if( e.attribute( "type" ) == "motor" ) {
				jtype = Motor;
			}
			else if( e.attribute( "type" ) == "manual" ) {
				jtype = Manual;
			}
			else {
				jtype = Free;
			}
			
			GenericJoint *gj = model->newGenericJoint(
				e.attribute("id"),
				NULL,
				jtype,
				e.attribute("tx", "0").toFloat(),
				e.attribute("tz", "0").toFloat(),
				e.attribute("ty", "0").toFloat(),
				e.attribute("rx", "0").toFloat(),
				e.attribute("rz", "0").toFloat(),
				e.attribute("ry", "0").toFloat(),
				e.attribute("port", "0").toInt() );
			imNode->addChild(gj);
			node = gj;
			gj->addChild( model->getNode( e.attribute("child") ) );
			gj->setParent( model->getNode( e.attribute("parent") ) );
		}
		else if (e.tagName().toLower() == "ball-joint") {
// 			Body * im = dynamic_cast<Body *>(imNode );
			JointType jtype;
			if( e.attribute( "type" ) == "motor" ) {
				jtype = Motor;
			}
			else if( e.attribute( "type" ) == "manual" ) {
				jtype = Manual;
			}
			else {
				jtype = Free;
			}
			
			BallJoint *bj = model->newBallJoint(
				e.attribute("id"),
				NULL,
				jtype,
				e.attribute("lx", "0").toFloat(),
				e.attribute("lz", "0").toFloat(),
				e.attribute("ly", "0").toFloat(),
				e.attribute("hx", "0").toFloat(),
				e.attribute("hz", "0").toFloat(),
				e.attribute("hy", "0").toFloat(),
				e.attribute("tx", "0").toFloat(),
				e.attribute("tz", "0").toFloat(),
				e.attribute("ty", "0").toFloat(),
				e.attribute("rx", "0").toFloat(),
				e.attribute("rz", "0").toFloat(),
				e.attribute("ry", "0").toFloat(),
				e.attribute("port", "0").toInt() );
			imNode->addChild(bj);
			node = bj;
			bj->addChild( model->getNode( e.attribute("child") ) );
			bj->setParent( model->getNode( e.attribute("parent") ) );
		}
		else if (e.tagName().toLower() == "hinge-joint") {
// 			Body * im = dynamic_cast<Body *>(imNode );
			JointType jtype;
			if( e.attribute( "type" ) == "motor" ) {
				jtype = Motor;
			}
			else if( e.attribute( "type" ) == "manual" ) {
				jtype = Manual;
			}
			else {
				jtype = Free;
			}
			
			HingeJoint *hj = model->newHingeJoint(
				e.attribute("id"),
				NULL,
				jtype,
				e.attribute("tx", "0").toFloat(),
				e.attribute("tz", "0").toFloat(),
				e.attribute("ty", "0").toFloat(),
				e.attribute("rx", "0").toFloat(),
				e.attribute("rz", "0").toFloat(),
				e.attribute("ry", "0").toFloat(),
				e.attribute("min", "-inf").toDouble(),
				e.attribute("max", "inf").toDouble(),
				e.attribute("axis","z").toStdString(),
				e.attribute("port", "0").toInt() );
			imNode->addChild(hj);
			node = hj;
			hj->addChild( model->getNode( e.attribute("child") ) );
			hj->setParent( model->getNode( e.attribute("parent") ) );
		}
		else if (e.tagName().toLower() == "prismatic-joint") {
// 			Body * im = dynamic_cast<Body *>(imNode );
			JointType jtype;
			if( e.attribute( "type" ) == "motor" ) {
				jtype = Motor;
			}
			else if( e.attribute( "type" ) == "manual" ) {
				jtype = Manual;
			}
			else {
				jtype = Free;
			}
			
			PrismaticJoint *pj = model->newPrismaticJoint(
				e.attribute("id"),
				NULL,
				jtype,
				e.attribute("tx", "0").toFloat(),
				e.attribute("tz", "0").toFloat(),
				e.attribute("ty", "0").toFloat(),
				e.attribute("rx", "0").toFloat(),
				e.attribute("rz", "0").toFloat(),
				e.attribute("ry", "0").toFloat(),
				e.attribute("min", "-inf").toDouble(),
				e.attribute("max", "inf").toDouble(),
				e.attribute("position", "0").toDouble(),
				e.attribute("offset", "0").toDouble(),
				e.attribute("axis","z").toStdString(),
				e.attribute("port", "0").toInt() );
			imNode->addChild(pj);
			node = pj;
			pj->addChild( model->getNode( e.attribute("child") ) );
			pj->setParent( model->getNode( e.attribute("parent") ) );
		}
		// ------------------------------------------------------------
		// Bodies start here
		// ------------------------------------------------------------
		else if (e.tagName().toLower() == "body") {
			QString btype = e.attribute( "type", "dynamic" ).toLower();
			
			// Read the body parameters
			BodyParams params; 
			if( btype == "static" ) {
				params.type = Static;
				params.mass = 0.0f;
			}
			else if( btype == "ghost" ) {
				params.type = Ghost;
				params.mass = NAN;
// 				params.restitution = NAN;
// 				params.friction = NAN;
			} else {
				params.type = Dynamic;
				params.mass = e.attribute("mass").toFloat();
			}
			params.restitution = e.attribute("restitution").toFloat();
			params.friction = e.attribute("friction").toFloat();
			
			// Read the body pose
			RTMat pose;
			pose.setTr( e.attribute("tx").toFloat(), e.attribute("tz").toFloat(), e.attribute("ty").toFloat() );
			pose.setR( e.attribute("rx").toFloat(), e.attribute("rz").toFloat(), e.attribute("ry").toFloat() );
			
			// Create the body
			Body* body = model->newBody( e.attribute("id"), imNode, pose, params );
			imNode->addChild(body);
			node = body;
		}
		else if (e.tagName().toLower() == "differential-robot") {
			GenericJoint * im = dynamic_cast<GenericJoint *>(imNode );
			DifferentialRobot *dr = model->newDifferentialRobot(
				e.attribute("id"),
				im,
				e.attribute("tx", "0").toFloat(),
				e.attribute("tz", "0").toFloat(),
				e.attribute("ty", "0").toFloat(),
				e.attribute("rx", "0").toFloat(),
				e.attribute("rz", "0").toFloat(),
				e.attribute("ry", "0").toFloat(),
				e.attribute("port", "0").toInt());
			imNode->addChild(dr);
			node = dr;
		}
		// ------------------------------------------------------------
		// Sensors start here
		// ------------------------------------------------------------
		else if (e.tagName().toLower() == "camera") {
			Camera *cam = model->newCamera(
				e.attribute("id"), imNode,
				e.attribute("width", "0").toFloat(),
				e.attribute("height", "0").toFloat(),
				e.attribute("focal", "0").toFloat());
			imNode->addChild(cam);
			node = cam;
		}
		else if (e.tagName().toLower() == "rgbd") {
			RGBD *cam = model->newRGBD(
				e.attribute("id"),
				imNode,
				e.attribute("width", "0").toFloat(),
				e.attribute("height", "0").toFloat(),
				e.attribute("focal", "0").toFloat(),
				e.attribute("port", "0").toInt(),
				e.attribute("ifconfig", ""));
			imNode->addChild(cam);
			node = cam;
		}
		else if (e.tagName().toLower() == "imu") {
			IMU *imu = model->newIMU(
				e.attribute("id"),
				imNode,
				e.attribute("port", "0").toInt());
			imNode->addChild(imu);
			node = imu;
		}
		else if (e.tagName().toLower() == "laser") {
			Laser *laser = model->newLaser(
				e.attribute("id"),
				imNode,
				e.attribute("port", "0").toInt(),
				e.attribute("min").toInt(),
				e.attribute("max").toInt(),
				e.attribute("angle").toFloat(),
				e.attribute("measures").toInt(),
				e.attribute("ifconfig"));
			imNode->addChild(laser);
			node = laser;
		}
		// ------------------------------------------------------------
		// Primitives start here
		// ------------------------------------------------------------
		else if (e.tagName().toLower() == "mesh") {
			// Get the parent
			Body* body = dynamic_cast<Body*>( imNode );
			if( body == NULL )
				qFatal( "Trying to add a mesh to something that is not a body!\n" );
			
			// Read the mesh parameters
			Mesh* mesh = new Mesh(
				e.attribute("id"),
				e.attribute("file"),
				e.attribute("tx", "0.0").toFloat(),
				e.attribute("tz", "0.0").toFloat(),
				e.attribute("ty", "0.0").toFloat(),
				e.attribute("rx", "0.0").toFloat(),
				e.attribute("rz", "0.0").toFloat(),
				e.attribute("ry", "0.0").toFloat(),
				e.attribute("scale", "1.0").toFloat(),
				e.attribute("scale", "1.0").toFloat(),
				e.attribute("scale", "1.0").toFloat(),
				body );
			
			body->addChild( mesh );
			mesh->setParent( body );
			node = mesh;
		}
		else if (e.tagName().toLower() == "plane")
		{
			QString size = e.attribute("size", "2500");
			QStringList li = size.split(",");
			float width = li[0].toFloat();
			float height = width;
			float depth = 0.05f;
			if (li.size() == 1) {
				height = width;
				depth = 0.05;
			}
			else if (li.size() == 2) {
				height = li[1].toFloat();
			}
			else if (li.size() == 3) {
				height = li[1].toFloat();
				depth = li[2].toFloat();
			}
			else {
				qFatal("too many numbers in plane definition");
			}
			Plane *plane = model->newPlane(
				e.attribute("id"),
				imNode,
				e.attribute("texture", ""),
				width,
				height,
				depth,
				e.attribute("repeat", "1000").toInt(),
				e.attribute("nx", "0").toFloat(),
				e.attribute("ny", "0").toFloat(),
				e.attribute("nz", "0").toFloat(),
				e.attribute("px", "0").toFloat(),
				e.attribute("py", "0").toFloat(),
				e.attribute("pz", "0").toFloat());
			imNode->addChild(plane);
			node = plane;
		}
		else if (e.tagName().toLower() == "pointcloud")
		{
			PointCloud *pointcloud = model->newPointCloud(
				e.attribute("id"),
				imNode);
			imNode->addChild(pointcloud);
			node = pointcloud;
		}
		// ------------------------------------------------------------
		// Invalid names
		// ------------------------------------------------------------
		else
		{
			qFatal("%s is not a valid tag name.\n", qPrintable(e.tagName()));
			return;
		}
		
		
		if( node != NULL ) {
// 			node->setParent(imNode);
			recursiveXML(domNode, model, node);
		}
	}
}



// ------------------------------------------------------------------------------------------------
// Viewer
// ------------------------------------------------------------------------------------------------

Viewer::Viewer( InnerModel *im, QString root, osg::Group *parent ) : osg::Switch()
{
	// Add a light source
	osg::Light* light1 = new osg::Light();
	light1->setLightNum( 1 );
	light1->setAmbient( osg::Vec4d(0.7, 0.7, 0.7, 1.0) );
	light1->setDiffuse( osg::Vec4d(0.7, 0.7, 0.7, 1.0) );
	light1->setSpecular( osg::Vec4d(0.7, 0.7, 0.7, 1.0) );
	light1->setPosition( osg::Vec4d(-3.0, 4.0, 8.0, 1.0) );
	light1->setQuadraticAttenuation( 0.001 );
	osg::LightSource* lightSource1 = new osg::LightSource();
	lightSource1->setLight( light1 );
	this->addChild( lightSource1 );
	osg::StateSet* stateset1 = this->getOrCreateStateSet();
	lightSource1->setStateSetModes( *stateset1, osg::StateAttribute::ON );
	
// 	osg::LightModel* lightmodel = new osg::LightModel; 
// 	lightmodel->setAmbientIntensity( osg::Vec4(0.2f,0.2f,0.2f,1.0f) );
// 	stateset1->setAttributeAndModes( lightmodel, osg::StateAttribute::ON );
	
// 	// Add a light source
// 	osg::Light* light2 = new osg::Light();
// 	light2->setLightNum( 2 );
// 	light2->setAmbient( osg::Vec4d(0.4, 0.4, 0.4, 1.0) );
// 	light2->setDiffuse( osg::Vec4d(0.4, 0.4, 0.4, 1.0) );
// 	light2->setSpecular( osg::Vec4d(0.4, 0.4, 0.4, 1.0) );
// 	light2->setPosition( osg::Vec4d(4.0, 6.0, 5.0, 1.0) );
// 	light2->setQuadraticAttenuation( 0.001 );
// 	osg::LightSource* lightSource2 = new osg::LightSource();
// 	lightSource2->setLight( light2 );
// 	this->addChild( lightSource2 );
// 	osg::StateSet* stateset2 = this->getOrCreateStateSet();
// 	lightSource2->setStateSetModes( *stateset2, osg::StateAttribute::ON );
	
	// Model construction
	innerModel = im;
	IM2::Node *imroot = innerModel->getNode(root);
	if ( imroot == NULL ) {
		throw Exception( "Viewer::Viewer(): Error: Specified root node not found." );
	}
	
	QQueue<IM2::Node*> pending;
	pending.enqueue( imroot );
	while( !pending.isEmpty() ) {
		IM2::Node* node = pending.dequeue();
// 		node->initPhysics( this );
		
		// Add a RGBD sensor
		if ( RGBD* rgbd = dynamic_cast<RGBD*>(node) ) {
			rgbds[ rgbd->id ] = rgbd;
		}
		// Add a camera
		else if ( Camera* camera = dynamic_cast<Camera*>(node) ) {
			cameras[ camera->id ] = camera;
		}
		// Add a laser
		else if ( Laser* laser = dynamic_cast<Laser*>(node) ) {
			lasers[ laser->id ] = laser;
		}
		// Add a laser
		else if ( IMU* imu = dynamic_cast<IMU*>(node) ) {
			imus[ imu->id ] = imu;
		}
		
		// Traverse the children nodes
		foreach( IM2::Node* child, node->im_children ) {
			pending.enqueue( child );
		}
	}
	
	// Update
	if (parent)
		parent->addChild(this);
}



Viewer::~Viewer()
{
}

};
