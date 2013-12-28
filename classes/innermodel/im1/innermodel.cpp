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
#include "innermodelreader.h"



// ------------------------------------------------------------------------------------------------
// InnerModelException
// ------------------------------------------------------------------------------------------------

InnerModelException::InnerModelException(const std::string &reason) : runtime_error(std::string("InnerModelException: ") + reason)
{
	std::cout << reason << std::endl;
}



// ------------------------------------------------------------------------------------------------
// InnerModelNode
// ------------------------------------------------------------------------------------------------

InnerModelNode::InnerModelNode(QString id_, InnerModelNode *parent_) : RTMat()
{
	fixed = true;
	parent = parent_;
	if (parent)
		level = parent->level+1;
	else
		level = 0;
	id = id_;
	attributes.clear();
}



void InnerModelNode::treePrint(QString s, bool verbose)
{
	printf("%s%s l(%d) [%d]\n", qPrintable(s), qPrintable(id), level, children.size());
	QList<InnerModelNode*>::iterator i;
	for (i=children.begin(); i!=children.end(); i++)
	{
		if (verbose)
			(*i)->print(verbose);
		(*i)->treePrint(s+QString("  "), verbose);
		
	}
}



void InnerModelNode::setParent(InnerModelNode *parent_)
{
	parent = parent_;
	level = parent->level+1;
}



void InnerModelNode::addChild(InnerModelNode *child)
{
	children.append(child);
	child->parent = this;
}



void InnerModelNode::setFixed(bool f)
{
	fixed = f;
}



bool InnerModelNode::isFixed()
{
	return fixed;
}



void InnerModelNode::updateChildren() {
	foreach(InnerModelNode *i, children)
		i->update();
}



// ------------------------------------------------------------------------------------------------
// InnerModel
// ------------------------------------------------------------------------------------------------

/// (Con/De)structors
InnerModel::InnerModel(std::string xmlFilePath)
{
	mutex = new QMutex(QMutex::Recursive);
	if (not InnerModelReader::load(QString::fromStdString(xmlFilePath), this))
	{
		qFatal("InnerModelReader::load error using file %s\n", xmlFilePath.c_str());
	}
}



InnerModel::InnerModel()
{
	// Set Mutex
	mutex = new QMutex(QMutex::Recursive);
	// Set Root node
	InnerModelTransform *root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	setRoot(root);
	hash["root"] = root;
	
	// How to use:
	//   InnerModelTransform *tr = innerModel->newTransform("name", parent, rx, ry, rz, px, py, pz);
	//   parent->addChild(tr);
}



InnerModel::InnerModel(const InnerModel &original)
{
}



InnerModel::~InnerModel()
{
}



///Remove sub tree and return sa list with his id
void InnerModel::removeSubTree(InnerModelNode *node, QStringList *l)
{
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		removeSubTree(*i,l);
	}
	node->parent->children.removeOne(node);
	l->append(node->id);
	removeNode(node->id);
}



///get sub tree and return a list with his id
void InnerModel::getSubTree(InnerModelNode *node, QStringList *l)
{
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		getSubTree(*i,l);
	}
	//node->parent->children.removeOne(node);
	l->append(node->id);
	//removeNode(node->id);
}



bool InnerModel::save(QString path)
{
	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return false;
	
	QTextStream out(&file);
	root->save(out, 0);
	file.close();
	return true;
}



InnerModel InnerModel::cloneFake(const QVec & basePose) const
{
	InnerModel rob( *this );
	rob.updateTransformValues("base", basePose(0), 0, basePose(1), 0, basePose(2), 0 );
	return rob;
}



/// Auto update method
void InnerModel::update()
{
	root->update();
	cleanupTables();
}



void InnerModel::cleanupTables()
{
	QMutexLocker l(mutex);
	localHashTr.clear();
	localHashRot.clear();
}



void InnerModel::setUpdateRotationPointers(QString rotationId, float *x, float *y, float *z)
{
	InnerModelTransform *aux;
	if ((aux=dynamic_cast<InnerModelTransform *>(hash[rotationId])) != NULL)
		aux->setUpdateRotationPointers(x, y, z);
	else if (hash[rotationId] == NULL)
		qDebug() << "There is no such" << rotationId << "node";
	else
		qDebug() << "Dynamic cast error from" << rotationId << "to InnerModelTransform. " << typeid(hash[rotationId]).name();
}



void InnerModel::setUpdateTranslationPointers(QString translationId, float *x, float *y, float *z)
{
	InnerModelTransform *aux;
	if ((aux=dynamic_cast<InnerModelTransform *>(hash[translationId])) != NULL)
		aux->setUpdateTranslationPointers(x, y, z);
	else if (hash[translationId] == NULL)
		qDebug() << "There is no such" << translationId << "node";
	else
		qDebug() << "Dynamic cast error from" << translationId << "to InnerModelTransform. " << typeid(hash[translationId]).name();
}



void InnerModel::setUpdateTransformPointers(QString transformId, float *tx, float *ty, float *tz, float *rx, float *ry, float *rz)
{
	InnerModelTransform *aux;
	if ((aux=dynamic_cast<InnerModelTransform *>(hash[transformId])) != NULL)
		aux->setUpdatePointers(tx, ty, tz,rx,ry,rz);
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
}



void InnerModel::setUpdatePlanePointers(QString planeId, float *nx, float *ny, float *nz, float *px, float *py, float *pz)
{
	InnerModelPlane *aux;
	if ((aux=dynamic_cast<InnerModelPlane *>(hash[planeId])) != NULL)
		aux->setUpdatePointers(nx, ny, nz, px, py, pz);
	else if (hash[planeId] == NULL)
		qDebug() << "There is no such" << planeId << "node";
	else
		qDebug() << "Dynamic cast error from" << planeId << "to InnerModelPlane";
}



void InnerModel::updateTransformValues(QString transformId, float tx, float ty, float tz, float rx, float ry, float rz, QString parentId)
{
	cleanupTables();
	
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform *>(hash[transformId]);
	if (aux != NULL)
	{
		if (parentId!="")
		{
			InnerModelTransform *auxParent = dynamic_cast<InnerModelTransform *>(hash[parentId]);
			if (auxParent!=NULL)
			{
				RTMat Tbi;
				Tbi.setTr( tx,ty,tz);
				Tbi.setR ( rx,ry,rz);
				
				///Tbp Inverse = Tpb. This gets Tpb directly. It's the same
				RTMat Tpb= getTransformationMatrix ( getNode ( transformId)->parent->id,parentId );
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
		aux->update(tx,ty,tz,rx,ry,rz);
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::updatePlaneValues(QString planeId, float nx, float ny, float nz, float px, float py, float pz)
{
	cleanupTables();
	
	InnerModelPlane *plane = dynamic_cast<InnerModelPlane *>(hash[planeId]);
	if (plane != NULL)
	{
		plane->update(nx, ny, nz, px, py, pz);
	}
	else if (hash[planeId] == NULL)
		qDebug() << "There is no such" << planeId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::updateTranslationValues(QString transformId, float tx, float ty, float tz, QString parentId)
{
	cleanupTables();
	
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform *>(hash[transformId]);
	if (aux != NULL)
	{
		if (parentId!="")
			updateTransformValues(transformId, tx,ty,tz,0.,0.,0.,parentId);
		else
			aux->update(tx,ty,tz,aux->backrX,aux->backrY,aux->backrZ);
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::updateRotationValues(QString transformId, float rx, float ry, float rz, QString parentId)
{
	cleanupTables();
	
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform *>(hash[transformId]);
	if (aux != NULL)
	{
		if (parentId!="")
		{
			updateTransformValues(transformId,0.,0.,0.,rx,ry,rz,parentId);
		}
		else
			aux->update(aux->backtX,aux->backtY,aux->backtZ,rx,ry,rz);
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
		qDebug() << "?????";
}



void InnerModel::updateJointValue(QString jointId, float angle)
{
	cleanupTables();
	
	InnerModelJoint *j = dynamic_cast<InnerModelJoint *>(hash[jointId]);
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
	
	InnerModelPrismaticJoint *j = dynamic_cast<InnerModelPrismaticJoint *>(hash[jointId]);
	if (j != NULL)
	{
		j->setPosition(pos);
	}
	else if (hash[jointId] == NULL)
		qDebug() << "There is no such" << jointId << "node";
	else
		qDebug() << "?????";
}



/// Model construction methods
void InnerModel::setRoot(InnerModelNode *node)
{
	root = node;
	hash["root"] = root;
	root->parent=NULL;
}



InnerModelTransform *InnerModel::newTransform(QString id, QString engine, InnerModelNode *parent, float tx, float ty, float tz, float rx, float ry, float rz, float mass)
{
	if (hash.contains(id)) qFatal("InnerModel::newTransform: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelTransform *newnode = new InnerModelTransform(id, engine, tx, ty, tz, rx, ry, rz, mass, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelJoint *InnerModel::newJoint(QString id, InnerModelTransform *parent,float lx, float ly, float lz,float hx, float hy, float hz, float tx, float ty, float tz, float rx, float ry, float rz, float min, float max, uint32_t port,std::string axis, float home)
{
	if (hash.contains(id)) qFatal("InnerModel::newJoint: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelJoint *newnode = new InnerModelJoint(id,lx,ly,lz,hx,hy,hz, tx, ty, tz, rx, ry, rz, min, max, port, axis, home, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelPrismaticJoint *InnerModel::newPrismaticJoint(QString id, InnerModelTransform *parent, float min, float max, float value, float offset, uint32_t port,std::string axis, float home)
{
	if (hash.contains(id)) qFatal("InnerModel::newPrismaticJoint: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelPrismaticJoint *newnode = new InnerModelPrismaticJoint(id, min, max, value, offset, port, axis, home, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelDifferentialRobot *InnerModel::newDifferentialRobot(QString id, InnerModelTransform *parent, float tx, float ty, float tz, float rx, float ry, float rz, uint32_t port)
{
	if (hash.contains(id)) qFatal("InnerModel::newDifferentialrobot: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelDifferentialRobot *newnode = new InnerModelDifferentialRobot(id, tx, ty, tz, rx, ry, rz, port, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelCamera *InnerModel::newCamera(QString id, InnerModelNode *parent, float width, float height, float focal)
{
	if (hash.contains(id)) qFatal("InnerModel::newCamera: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelCamera *newnode = new InnerModelCamera(id, width, height, focal, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelRGBD *InnerModel::newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, float noise, uint32_t port, QString ifconfig)
{
	if (noise < 0)
		qFatal("InnerModel::newRGBD: noise can't have negative values");
	if (hash.contains(id)) qFatal("InnerModel::newRGBD: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelRGBD *newnode = new InnerModelRGBD(id, width, height, focal, noise, port, ifconfig, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelIMU *InnerModel::newIMU(QString id, InnerModelNode *parent, uint32_t port)
{
	if (hash.contains(id)) qFatal("InnerModel::newIMU: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	// 	printf("newIMU id=%s  parentId=%s port=%d\n", id.toStdString().c_str(), parent->id.toStdString().c_str(), port);
	InnerModelIMU *newnode = new InnerModelIMU(id, port, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelLaser *InnerModel::newLaser(QString id, InnerModelNode *parent, uint32_t port, uint32_t min, uint32_t max, float angle, uint32_t measures, QString ifconfig)
{
	if (hash.contains(id)) qFatal("InnerModel::newLaser: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	// 	printf("newLaser id=%s  parentId=%s port=%d min=%d max=%d angle=%f measures=%d\n", id.toStdString().c_str(), parent->id.toStdString().c_str(), port, min, max, angle, measures);
	InnerModelLaser *newnode = new InnerModelLaser(id, port, min, max, angle, measures, ifconfig, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelPlane *InnerModel::newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx, float ny, float nz, float px, float py, float pz)
{
	if (hash.contains(id)) qFatal("InnerModel::newPlane: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelPlane *newnode = new InnerModelPlane(id, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelMesh *InnerModel::newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz)
{
	if (hash.contains(id)) qFatal("InnerModel::newMesh: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelMesh *newnode = new InnerModelMesh(id, path, scalex, scaley, scalez, (InnerModelMesh::RenderingModes)render, tx, ty, tz, rx, ry, rz, parent);
	hash[id] = newnode;
	return newnode;
}



InnerModelMesh *InnerModel::newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz)
{
	return newMesh(id,parent,path,scale,scale,scale,render,tx,ty,tz,rx,ry,rz);
}



InnerModelPointCloud *InnerModel::newPointCloud(QString id, InnerModelNode *parent)
{
	if (hash.contains(id)) qFatal("InnerModel::newPointCloud: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
	InnerModelPointCloud *newnode = new InnerModelPointCloud(id, parent);
	hash[id] = newnode;
	printf("Inserted point cloud %s ptr(%p), on node %s\n", id.toStdString().c_str(), newnode, parent->id.toStdString().c_str());
	return newnode;
}



InnerModelTransform *InnerModel::getTransform(const QString &id)
{
	InnerModelTransform *tr = dynamic_cast<InnerModelTransform *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such transform %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a transform", id.toStdString().c_str());
	}
	return tr;
}



InnerModelJoint *InnerModel::getJoint(const QString &id)
{
	InnerModelJoint *tr = dynamic_cast<InnerModelJoint *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such joint %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a joint", id.toStdString().c_str());
	}
	return tr;
}



InnerModelPrismaticJoint *InnerModel::getPrismaticJoint(const QString &id)
{
	InnerModelPrismaticJoint *tr = dynamic_cast<InnerModelPrismaticJoint *>(hash[id]);
	if (not tr)
	{
		if (not hash[id])
			qFatal("No such joint %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a prismatic joint", id.toStdString().c_str());
	}
	return tr;
}



InnerModelCamera *InnerModel::getCamera(const QString id)
{
	InnerModelCamera *camera = dynamic_cast<InnerModelCamera *>(hash[id]);
	if (not camera)
	{
		if (not hash[id])
			qFatal("No such camera %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a camera", id.toStdString().c_str());
	}
	return camera;
}



InnerModelRGBD *InnerModel::getRGBD(const QString id)
{
	InnerModelRGBD *camera = dynamic_cast<InnerModelRGBD *>(hash[id]);
	if (not camera)
	{
		if (not hash[id])
			qFatal("No such camera %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a camera", id.toStdString().c_str());
	}
	return camera;
}



InnerModelIMU *InnerModel::getIMU(const QString id)
{
	InnerModelIMU *imu = dynamic_cast<InnerModelIMU *>(hash[id]);
	if (not imu)
	{
		if (not hash[id])
			qFatal("No such innertial unit %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be an innertial unit", id.toStdString().c_str());
	}
	return imu;
}



InnerModelLaser *InnerModel::getLaser(const QString id)
{
	InnerModelLaser *laser = dynamic_cast<InnerModelLaser *>(hash[id]);
	if (not laser)
	{
		if (not hash[id])
			qFatal("No such laser %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be an laser", id.toStdString().c_str());
	}
	return laser;
}



InnerModelPlane *InnerModel::getPlane(const QString &id)
{
	InnerModelPlane *plane = dynamic_cast<InnerModelPlane *>(hash[id]);
	if (not plane)
	{
		if (not hash[id])
			qFatal("No such plane %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a plane", id.toStdString().c_str());
	}
	return plane;
}



InnerModelMesh *InnerModel::getMesh(const QString &id)
{
	InnerModelMesh *mesh = dynamic_cast<InnerModelMesh *>(hash[id]);
	if (not mesh)
	{
		if (not hash[id])
			qFatal("No such mesh %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a mesh", id.toStdString().c_str());
	}
	return mesh;
}



InnerModelPointCloud *InnerModel::getPointCloud(const QString &id)
{
	InnerModelPointCloud *pointcloud = dynamic_cast<InnerModelPointCloud *>(hash[id]);
	if (not pointcloud)
	{
		if (not hash[id])
			qFatal("No such pointcloud %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a pointcloud", id.toStdString().c_str());
	}
	return pointcloud;
}



InnerModelDifferentialRobot *InnerModel::getDifferentialRobot(const QString &id)
{
	InnerModelDifferentialRobot *diff = dynamic_cast<InnerModelDifferentialRobot *>(hash[id]);
	if (not diff)
	{
		if (not hash[id])
			qFatal("No such differential robot %s", id.toStdString().c_str());
		else
			qFatal("%s doesn't seem to be a differential robot", id.toStdString().c_str());
	}
	return diff;
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
	this->fundamental.set( essential, dynamic_cast<InnerModelCamera *>(hash[firstCam])->camera, dynamic_cast<InnerModelCamera *>(hash[secondCam])->camera );
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
	
	QVec pI = this->getRotationMatrixTo("central", "firstCam") * static_cast<InnerModelCamera *>(hash[firstCam])->camera.getRayHomogeneous( first );
	QVec pD = this->getRotationMatrixTo("central", "secondCam") * static_cast<InnerModelCamera *>(hash[secondCam])->camera.getRayHomogeneous( second );
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
	InnerModelCamera *camera=NULL;
	
	camera = dynamic_cast<InnerModelCamera *>(hash[cameraId]);
	if (not camera)
		qFatal("No such %s camera", qPrintable(cameraId));
	
	pc = camera->camera.project(origVec);
	
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
		QVec p = static_cast<InnerModelCamera *>(hash[cameraId])->camera.getRayHomogeneous(coord);
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
	InnerModelPlane *plane = getPlane(planeId);
	InnerModelCamera *camera = getCamera(cameraId);
	// Transform rotate plane normal vector to camera reference system
	QMat rtm = getRotationMatrixTo(cameraId, planeId);
	QVec vec = QVec::vec3(plane->normal(0), plane->normal(1), plane->normal(2));
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
	
	if (abs(dx) <= 1)
	{
		if (abs(dy) <= 1) qFatal("Degenerated camera");
		return QVec::vec3(-1, 0, p1(0));
	}
	else
	{
		return QVec::vec3(dy/dx, -1, camera->camera.getHeight()-(p1(1)-(dy*p1(0)/dx))+heightOffset);
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
		foreach (InnerModelNode *i, listA)
		{
			ret = ((RTMat)(*i)).operator*(ret);
		}
		foreach (InnerModelNode *i, listB)
		{
			ret = i->invert() * ret;
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
		InnerModelTransform *tf=NULL;
		
		foreach (InnerModelNode *i, listA)
		{
			if ((tf=dynamic_cast<InnerModelTransform *>(i))!=NULL)
			{
				rret = tf->getR() * rret;
			}
		}
		foreach (InnerModelNode *i, listB)
		{
			if ((tf=dynamic_cast<InnerModelTransform *>(i))!=NULL)
			{
				rret = tf->getR().transpose() * rret;
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
	QVec planeN = getPlane(plane)->normal;
	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = transform(sourceCamera, getPlane(plane)->point, plane);
	
	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = getCamera(sourceCamera)->camera;
	QMat K2 = getCamera(virtualCamera)->camera;
	
	double d = -(planePoint*planeN);
	QMat H = K2 * ( R - ((t*n.transpose()) / d) ) * K1.invert();
	return H;
}



QMat InnerModel::getAffineHomographyMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	QVec planeN = getPlane(plane)->normal;
	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = transform(sourceCamera, getPlane(plane)->point, plane);
	
	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = getCamera(sourceCamera)->camera;
	
	double d = -(planePoint*planeN);
	QMat H = ( R - ((t*n.transpose()) / d) ) * K1.invert();
	for (int r=0;r<2;r++)
		for (int c=0;c<3;c++)
			H(r,c) = H(r,c) * 1000.;
		return H;
}



QMat InnerModel::getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	QVec planeN = getPlane(plane)->normal;
	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = transform(sourceCamera, getPlane(plane)->point, plane);
	
	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = getCamera(sourceCamera)->camera;
	
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
	InnerModelNode *a=hash[origId], *b=hash[destId];
	if (!a)
		throw InnerModelException("Cannot find node: \""+ origId.toStdString()+"\"");
	if (!b)
		throw InnerModelException("Cannot find node: "+ destId.toStdString()+"\"");
	
	int minLevel = a->level<b->level? a->level : b->level;
	listA.clear();
	while (a->level >= minLevel)
	{
		listA.push_back(a);
		if(a->parent == NULL)
		{
			// 			qFatal("InnerModel::setLists: It wouldn't be here!!!!");
			break;
		}
		a=a->parent;
	}
	
	listB.clear();
	while (b->level >= minLevel)
	{
		listB.push_front(b);
		if(b->parent == NULL)
		{
			// 			qFatal("InnerModel::setLists: It wouldn't be here!!!!");
			break;
		}
		b=b->parent;
	}
	while (b!=a)
	{
		listA.push_back(a);
		listB.push_front(b);
		a = a->parent;
		b = b->parent;
	}
}



/// Robex Base specific getters
float InnerModel::getCameraFocal(const QString & cameraId) const
{
	return static_cast<InnerModelCamera *>(getNode(cameraId))->camera.getFocal();
}



int InnerModel::getCameraWidth(QString cameraId)
{
	return getCamera(cameraId)->camera.getWidth();
}



int InnerModel::getCameraHeight(const QString & cameraId) const
{
	return static_cast<InnerModelCamera *>(getNode(cameraId))->camera.getHeight();
}



int InnerModel::getCameraSize(const QString & cameraId) const
{
	return static_cast<InnerModelCamera *>(getNode(cameraId))->camera.getSize();
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



// ------------------------------------------------------------------------------------------------
// InnerModelTransform
// ------------------------------------------------------------------------------------------------

InnerModelTransform::InnerModelTransform(QString id_, QString engine_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float mass_, InnerModelNode *parent_)
: InnerModelNode(id_, parent_)
{
	engine = engine_;
	set(rx_, ry_, rz_, tx_, ty_, tz_);
	mass = mass_;
	backtX = tx_;
	backtY = ty_;
	backtZ = tz_;
	backrX = rx_;
	backrY = ry_;
	backrZ = rz_;
	rx = ry = rz = tx = ty = tz = NULL;
	gui_translation = gui_rotation = true;
}



void InnerModelTransform::print(bool verbose)
{
	printf("Transform: %s\n", qPrintable(id));
	if (verbose)
	{
		((QMat *)this)->print(qPrintable(id));
		getTr().print(id+"_T");
		//extractAnglesR().print(id+"_R");
	}
}



void InnerModelTransform::save(QTextStream &out, int tabs)
{
	QList<InnerModelNode*>::iterator c;
	
	if (id == "root")
	{
		for (int i=0; i<tabs; i++) out << "\t";
		out << "<innermodel>\n";
		for (c=children.begin(); c!=children.end(); c++) (*c)->save(out, tabs+1);
		for (int i=0; i<tabs; i++) out << "\t";
		out << "</innermodel>\n";
	}
	else
	{
		for (int i=0; i<tabs; i++) out << "\t";
		if (gui_translation and not gui_rotation)
			out << "<translation id=\"" << id << "\" tx=\""<< QString::number(backtX, 'g', 10) <<"\" ty=\""<< QString::number(backtY, 'g', 10) <<"\" tz=\""<< QString::number(backtZ, 'g', 10) <<"\">\n";
		else if (gui_rotation and not gui_translation)
			out << "<rotation id=\"" << id << "\" tx=\""<< QString::number(backrX, 'g', 10) <<"\" ty=\""<< QString::number(backrY, 'g', 10) <<"\" tz=\""<< QString::number(backrZ, 'g', 10) <<"\">\n";
		else
			out << "<transform id=\"" << id << "\" tx=\""<< QString::number(backtX, 'g', 10) <<"\" ty=\""<< QString::number(backtY, 'g', 10) <<"\" tz=\""<< QString::number(backtZ, 'g', 10) <<"\"  rx=\""<< QString::number(backrX, 'g', 10) <<"\" ry=\""<< QString::number(backrY, 'g', 10) <<"\" rz=\""<< QString::number(backrZ, 'g', 10) <<"\">\n";
					
		for (c=children.begin(); c!=children.end(); c++)
			(*c)->save(out, tabs+1);

		for (int i=0; i<tabs; i++) out << "\t";
		if (gui_translation and not gui_rotation )
			out << "</translation>\n";
		else if (gui_rotation and not gui_translation)
			out << "</rotation>\n";
		else
			out << "</transform>\n";
	}
}



void InnerModelTransform::setUpdatePointers(float *tx_, float *ty_, float *tz_, float *rx_, float *ry_, float *rz_)
{
	tx = tx_;
	ty = ty_;
	tz = tz_;
	rx = rx_;
	ry = ry_;
	rz = rz_;
	fixed = false;
}



void InnerModelTransform::setUpdateTranslationPointers(float *tx_, float *ty_, float *tz_)
{
	tx = tx_;
	ty = ty_;
	tz = tz_;
	fixed = false;
}



void InnerModelTransform::setUpdateRotationPointers(float *rx_, float *ry_, float *rz_)
{
	rx = rx_;
	ry = ry_;
	rz = rz_;
	fixed = false;
}



void InnerModelTransform::update()
{
	if (!fixed)
	{
		if (tx) backtX = *tx;
		if (ty) backtY = *ty;
		if (tz) backtZ = *tz;
		if (rx) backrX = *rx;
		if (ry) backrY = *ry;
		if (rz) backrZ = *rz;
		set(backrX, backrY, backrZ, backtX, backtY, backtZ);
	}
	updateChildren();
}



void InnerModelTransform::update(float tx_, float ty_, float tz_, float rx_, float ry_, float rz_)
{
	backrX = rx_; backrY = ry_; backrZ = rz_;
	backtX = tx_; backtY = ty_; backtZ = tz_;
	set(backrX, backrY, backrZ, backtX, backtY, backtZ);
	fixed = true;
}



// ------------------------------------------------------------------------------------------------
// InnerModelJoint
// ------------------------------------------------------------------------------------------------

InnerModelJoint::InnerModelJoint(QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_, float max_, uint32_t port_, std::string axis_, float home_, InnerModelTransform *parent_)
: InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
{
// 		set(rx_, ry_, rz_, tx_, ty_, tz_);
	backlX = lx_;
	backlY = ly_;
	backlZ = lz_;
	backhX = hx_;
	backhY = hy_;
	backhZ = hz_;
	min = min_;
	max = max_;
	home = home_;
	hx = hy = hz =lx = ly = lz = NULL;
	port = port_;
	axis = axis_;
	if (axis == "x")
	{
		update(min, 0, 0, max, 0, 0);
	}
	else if (axis == "y")
	{
		update(0, min, 0, 0, max, 0);
	}
	else if (axis == "z")
	{
		update(0, 0, min, 0, 0, max);
	}
	else
	{
		qFatal("internal error, no such axis %s\n", axis.c_str());
	}
}



void InnerModelJoint::print(bool verbose)
{
	printf("Joint: %s\n", qPrintable(id));
	if (verbose)
	{
		((QMat *)this)->print(qPrintable(id));
		getTr().print(id+"_T");
		//extractAnglesR().print(id+"_R");
	}
}



void InnerModelJoint::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "### joints cannot be saved yet ###\n";
}



void InnerModelJoint::setUpdatePointers(float *lx_, float *ly_, float *lz_, float *hx_, float *hy_, float *hz_)
{
	lx = lx_;
	ly = ly_;
	lz = lz_;
	hx = hx_;
	hy = hy_;
	hz = hz_;
	fixed = false;
}



void InnerModelJoint::update()
{
	if (!fixed)
	{
		if (lx) backtX = *tx;
		if (ly) backtY = *ty;
		if (lz) backtZ = *tz;
		if (rx) backhX = *hx;
		if (ry) backhY = *hy;
		if (rz) backhZ = *hz;
	}
	updateChildren();
}



void InnerModelJoint::update(float lx_, float ly_, float lz_, float hx_, float hy_, float hz_)
{
	backhX = hx_; backhY = hy_; backhZ = hz_;
	backlX = lx_; backlY = ly_; backlZ = lz_;
	fixed = true;
}



float InnerModelJoint::getAngle()
{
	return backrZ;
}



float InnerModelJoint::setAngle(float angle)
{
	float ret;
	if (angle <= max and angle >= min)
	{
		ret = angle;
	}
	else if (angle > max)
	{
		ret = max;
	}
	else
	{
		ret = min;
	}
	backrZ = ret;
	if (axis == "x")
	{
		set(ret,0,0, 0,0,0);
	}
	else if (axis == "y")
	{
		set(0,ret,0, 0,0,0);
	}
	else if (axis == "z")
	{
		set(0,0,ret, 0,0,0);
	}
	else
	{
		qFatal("internal error, no such axis %s\n", axis.c_str());
		return 0;
	}
	return ret;
}



QVec InnerModelJoint::unitaryAxis()
{
	if( axis == "x") return QVec::vec3(1,0,0);
	if( axis == "y") return QVec::vec3(0,1,0);
	if( axis == "z") return QVec::vec3(0,0,1);
	return QVec::zeros(3);
}



// ------------------------------------------------------------------------------------------------
// InnerModelPrismaticJoint
// ------------------------------------------------------------------------------------------------

InnerModelPrismaticJoint::InnerModelPrismaticJoint(QString id_, float min_, float max_, float val_, float offset_, uint32_t port_, std::string axis_, float home_, InnerModelTransform *parent_)
: InnerModelTransform(id_,QString("static"),0,0,0,0,0,0, 0, parent_)
{
	min = min_;
	max = max_;
	port = port_;
	axis = axis_;
	home = home_;
	offset = offset_;
	fixed = false;
	setPosition(val_);
}



void InnerModelPrismaticJoint::print(bool verbose)
{
	printf("Prismatic Joint: %s\n", qPrintable(id));
	if (verbose)
	{
		((QMat *)this)->print(qPrintable(id));
		getTr().print(id+"_T");
	}
}



void InnerModelPrismaticJoint::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "### joints cannot be saved yet ###\n";
}



void InnerModelPrismaticJoint::update()
{
	updateChildren();
}



float InnerModelPrismaticJoint::getPosition()
{
	return value;
}



float InnerModelPrismaticJoint::setPosition(float v)
{
	float ret;
	if (v <= max and v >= min)
	{
		ret = v;
	}
	else
	{
		if (v > max)
			ret = max;
		else
			ret = min;
	}
	value = v = ret;
	if (axis == "x")
	{
		set(0,0,0, v+offset,0,0);
	}
	else if (axis == "y")
	{
		set(0,0,0, 0,v+offset,0);
	}
	else if (axis == "z")
	{
		set(0,0,0, 0,0,v+offset);
	}
	else
	{
		qFatal("internal error, no such axis %s\n", axis.c_str());
	}
	return ret;
}



// ------------------------------------------------------------------------------------------------
// InnerModelDifferentialRobot
// ------------------------------------------------------------------------------------------------

InnerModelDifferentialRobot::InnerModelDifferentialRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_, InnerModelTransform *parent_)
: InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
{
	port = port_;
}



// ------------------------------------------------------------------------------------------------
// InnerModelPlane
// ------------------------------------------------------------------------------------------------

InnerModelPlane::InnerModelPlane(QString id_, QString texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, InnerModelNode *parent_)
: InnerModelNode(id_, parent_)
{
	normal = QVec::vec3(nx_, ny_, nz_);
	point = QVec::vec3(px_, py_, pz_);
	nx = ny = nz = px = py = pz = NULL;
	texture = texture_;
	width = width_;
	height = height_;
	depth = depth_;
	repeat = repeat_;
}



void InnerModelPlane::print(bool verbose)
{
	if (verbose) normal.print(QString("Plane: ")+id);
}



void InnerModelPlane::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<plane id=\"" << id << "\" texture=\"" << texture << "\" repeat=\"" << QString::number(repeat, 'g', 10) << "\" nx=\"" << QString::number(normal(0), 'g', 10) << "\" ny=\"" << QString::number(normal(1), 'g', 10) << "\" nz=\"" << QString::number(normal(2), 'g', 10) << "\" px=\"" << QString::number(point(0), 'g', 10) << "\" py=\"" << QString::number(point(1), 'g', 10) << "\" pz=\"" << QString::number(point(2), 'g', 10) << "\" />\n";
}



void InnerModelPlane::setUpdatePointers(float *nx_, float *ny_, float *nz_, float *px_, float *py_, float *pz_)
{
	nx = nx_;
	ny = ny_;
	nz = nz_;
	px = px_;
	py = py_;
	pz = pz_;
	nx = ny = nz = px = py = pz = NULL;
	fixed = false;
}



void InnerModelPlane::update()
{
	if (!fixed)
	{
		update(nx!=NULL?*nx:normal(0), ny!=NULL?*ny:normal(1), nz!=NULL?*nz:normal(2), px!=NULL?*px:point(0), py!=NULL?*py:point(1), pz!=NULL?*pz:point(2));
	}
	updateChildren();
}



void InnerModelPlane::update(float nx_, float ny_, float nz_, float px_, float py_, float pz_)
{
	normal(0) = nx_;
	normal(1) = ny_;
	normal(2) = nz_;
	point(0) = px_;
	point(1) = py_;
	point(2) = pz_;
	fixed = true;
}



// ------------------------------------------------------------------------------------------------
// InnerModelCamera
// ------------------------------------------------------------------------------------------------

InnerModelCamera::InnerModelCamera(QString id_, float width_, float height_, float focal_, InnerModelNode *parent_)
: InnerModelNode(id_, parent_)
{
	camera = Cam(focal, focal, width/2., height/2.);
	camera.setSize(width, height);
	camera.print(id_);
	width = width_;
	height = height_;
	focal = focal_;
}



void InnerModelCamera::print(bool verbose)
{
	if (verbose) camera.print(QString("Camera: ")+id);
}



void InnerModelCamera::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<camera id=\"" << id << "\" width=\"" << camera.getWidth() << "\" height=\"" << camera.getHeight() << "\" focal=\"" << QString::number(camera.getFocal(), 'g', 10) << "\" />\n";
}



void InnerModelCamera::update()
{
	if (fixed)
	{
	}
	updateChildren();
}



// ------------------------------------------------------------------------------------------------
// InnerModelIMU
// ------------------------------------------------------------------------------------------------

InnerModelRGBD::InnerModelRGBD(QString id_, float width, float height, float focal, float _noise, uint32_t _port, QString _ifconfig, InnerModelNode *parent_) : InnerModelCamera(id_, width, height, focal, parent_)
{
	noise = _noise;
	printf("InnerModelRGBD: %f {%d}\n", noise, port);
	port = _port;
	ifconfig = _ifconfig;
}



void InnerModelRGBD::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<rgbd id=\"" << id << "\" width=\"" << camera.getWidth() << "\" height=\"" << camera.getHeight() << "\" focal=\"" << QString::number(camera.getFocal(), 'g', 10) << "\" />\n";
}



// ------------------------------------------------------------------------------------------------
// InnerModelIMU
// ------------------------------------------------------------------------------------------------

InnerModelIMU::InnerModelIMU(QString id_, uint32_t _port, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
	port = _port;
}



void InnerModelIMU::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<imu id=\"" << id << "\" />\n";
}



void InnerModelIMU::print(bool verbose)
{
	if (verbose) printf("IMU.");
}



void InnerModelIMU::update()
{
	if (fixed)
	{
	}
	updateChildren();
}



// ------------------------------------------------------------------------------------------------
// InnerModelLaser
// ------------------------------------------------------------------------------------------------

InnerModelLaser::InnerModelLaser(QString id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, QString _ifconfig, InnerModelNode *parent_)
: InnerModelNode(id_, parent_)
{
	port = _port;
	min = _min;
	max = _max;
	measures = _measures;
	angle = _angle;
	ifconfig = _ifconfig;
}



void InnerModelLaser::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<laser id=\"" << id << "\" />\n";
}



void InnerModelLaser::print(bool verbose)
{
	if (verbose) printf("LASER.");
}



void InnerModelLaser::update()
{
	if (fixed)
	{
	}
	updateChildren();
}



// ------------------------------------------------------------------------------------------------
// InnerModelMesh
// ------------------------------------------------------------------------------------------------

InnerModelMesh::InnerModelMesh(QString id_, QString meshPath_, float scale, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, InnerModelNode *parent_)
: InnerModelNode(id_, parent_)
{
	InnerModelMesh(id_,meshPath_,scale,scale,scale,render_,tx_,ty_,tz_,rx_,ry_,rz_,parent_);
}



InnerModelMesh::InnerModelMesh(QString id_, QString meshPath_, float scalex_, float scaley_, float scalez_, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, InnerModelNode *parent_)
: InnerModelNode(id_, parent_)
{
	id = id_;
	render = render_;
	meshPath = meshPath_;
	scalex = scalex_;
	scaley = scaley_;
	scalez = scalez_;
	tx = tx_;
	ty = ty_;
	tz = tz_;
	rx = rx_;
	ry = ry_;
	rz = rz_;
}



void InnerModelMesh::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<mesh id=\""<<id<<"\"" <<" file=\"" << meshPath << "\" scale=\"" << QString::number(scalex, 'g', 10) << ","<< QString::number(scaley, 'g', 10)<< ","<< QString::number(scalez, 'g', 10) << "\" tx=\"" << QString::number(tx, 'g', 10) << "\" ty=\"" << QString::number(ty, 'g', 10) << "\" tz=\"" << QString::number(tz, 'g', 10) << "\" rx=\"" << QString::number(rx, 'g', 10) << "\" ry=\"" << QString::number(ry, 'g', 10) << "\" rz=\"" << QString::number(rz, 'g', 10) << "\" />\n";
}



void InnerModelMesh::print(bool verbose)
{
	if (verbose) printf("Mesh: %s\n", qPrintable(id));
}



void InnerModelMesh::update()
{
	if (fixed)
	{
	}
	updateChildren();
}



void InnerModelMesh::setScale(float x, float y, float z)
{
	scalex=x;
	scaley=y;
	scalez=z;
}



bool InnerModelMesh::normalRendering() const
{
	return render == NormalRendering;
}



bool InnerModelMesh::wireframeRendering() const {
	return render == WireframeRendering;
}



// ------------------------------------------------------------------------------------------------
// InnerModelPointCloud
// ------------------------------------------------------------------------------------------------

InnerModelPointCloud::InnerModelPointCloud(QString id_, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
	id = id_;
}



void InnerModelPointCloud::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<pointcloud id=\""<<id<<"\"/>\n";
}



void InnerModelPointCloud::print(bool verbose)
{
	if (verbose) printf("Point Cloud: %s\n", qPrintable(id));
}



void InnerModelPointCloud::update()
{
	if (fixed)
	{
	}
	updateChildren();
}
