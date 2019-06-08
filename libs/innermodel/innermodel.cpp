/*
 *    Copyright (C) 2010-2015 by RoboLab - University of Extremadura
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

#include <innermodel/innermodel.h>
#include <innermodel/innermodelreader.h>

// #include <osg/io_utils>
// #include <osgDB/ReadFile>
// #include <osg/Geode>

#include <iostream>
#include <fstream>

bool InnerModel::support_fcl()
{
#if FCL_SUPPORT==1
	return true;
#else
	return false;
#endif
}

///////////////////////
/// (Con/De)structors
///////////////////////
InnerModel::InnerModel(std::string xmlFilePath)
{
	//QMutexLocker ml(mutex);
	root = NULL;
	if (not InnerModelReader::load(QString::fromStdString(xmlFilePath), this))
	{
		QString error;
		error.sprintf("InnerModelReader::load error using file %s\n", xmlFilePath.c_str());
		throw error;
	}
}

InnerModel::InnerModel()
{
	// Set Mutex
	
	// Set Root node
	InnerModelTransform *root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	root->parent = NULL;
	setRoot(root);
	root->innerModel = this;
	hash["root"] = root;
}

InnerModel::InnerModel(const InnerModel &original)
{
	
	root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	setRoot(root);
	root->innerModel = this;
	hash["root"] = root;

	QList<InnerModelNode *>::iterator i;
	for (i=original.root->children.begin(); i!=original.root->children.end(); i++)
	{
		root->addChild((*i)->copyNode(hash, root));
	}
}

InnerModel::InnerModel(InnerModel &original)
{
	
	root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	setRoot(root);
	root->innerModel = this;
	hash["root"] = root;

	QList<InnerModelNode *>::iterator i;
	for (i=original.root->children.begin(); i!=original.root->children.end(); i++)
	{
		root->addChild((*i)->copyNode(hash, root));
	}
}

InnerModel::InnerModel(InnerModel *original)
{
	
	root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	setRoot(root);
	root->innerModel = this;
	hash["root"] = root;

	QList<InnerModelNode *>::iterator i;
	for (i=original->root->children.begin(); i!=original->root->children.end(); i++)
	{
		root->addChild((*i)->copyNode(hash, root));
	}
}

InnerModel::~InnerModel()
{
	foreach (QString id, getIDKeys())
	{
		InnerModelNode *dd = hash[id];
		delete dd;
	}
	hash.clear();
	localHashRot.clear();
	localHashTr.clear();
	listA.clear();
	listB.clear();
}

InnerModel* InnerModel::copy()
{
	InnerModel *inner = new InnerModel();
	QList<InnerModelNode *>::iterator i;
	for (i=root->children.begin(); i!=root->children.end(); i++)
		inner->root->addChild((*i)->copyNode(inner->hash, inner->root));
	return inner;
}

void InnerModel::removeNode(const QString & id)
{
	InnerModelNode *dd = hash[id];
	delete dd;
	hash.remove(id);
}

bool InnerModel::open(std::string xmlFilePath)
{
	return InnerModelReader::load(QString::fromStdString(xmlFilePath), this);
}

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

/**
 * @brief Returns a list of node's ID corresponding to the subtree starting at node
 *
 * @param node starting node
 * @param l pointer to QStringList
 * @return void
 */
void InnerModel::getSubTree(InnerModelNode *node, QStringList *l)
{
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		getSubTree(*i,l);
	}
	l->append(node->id);
}

void InnerModel::getSubTree(InnerModelNode *node, QList<InnerModelNode *> *l)
{
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		l->append((*i));
		getSubTree(*i,l);
	}
}

/**
 * @brief Returns a list of node's ID corresponding to the subtree starting at node
 *
 * @param node starting node
 * @param l pointer to QStringList
 * @return void
 */
void InnerModel::moveSubTree(InnerModelNode *nodeSrc, InnerModelNode *nodeDst)
{
	nodeSrc->parent->children.removeOne(nodeSrc);
	nodeDst->addChild(nodeSrc);
	nodeSrc->setParent(nodeDst);
	computeLevels(nodeDst);
}

void InnerModel::computeLevels(InnerModelNode *node)
{
	if (node->parent != NULL )
	{
		node->level=node->parent->level+1;
	}
	QList<InnerModelNode*>::iterator i;
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		computeLevels(*i);
	}
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


/// Auto update method
void InnerModel::update()
{
	root->update();
	cleanupTables();
}

void InnerModel::cleanupTables()
{
		localHashTr.clear();
		localHashRot.clear();
}

void InnerModel::ChangeHash(QString new_id, InnerModelNode *node)
{
	hash[new_id] = node;
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
				Tbi.setTr(tx,ty,tz);
				Tbi.setR (rx,ry,rz);

				///Tbp Inverse = Tpb. This gets Tpb directly. It's the same
				RTMat Tpb = getTransformationMatrix ( getNode ( transformId)->parent->id,parentId );
				///New Tpi
				RTMat Tpi = Tpb*Tbi;

				QVec angles = Tpi.extractAnglesR();
				QVec tr = Tpi.getTr();

				rx = angles.x();
				ry = angles.y();
				rz = angles.z();
				tx = tr.x();
				ty = tr.y();
				tz = tr.z();
			}
			else if (hash[parentId] == NULL)
			{
				qDebug() << "There is no such" << parentId << "node";
			}
		}
		//always update
		aux->update(tx,ty,tz,rx,ry,rz);
	}
	else if (hash[transformId] == NULL)
	{
		qDebug() << "There is no such" << transformId << "node";
	}
}

void InnerModel::updateTransformValuesS(std::string transformId, float tx, float ty, float tz, float rx, float ry, float rz, std::string parentId)
{
		updateTransformValues(QString::fromStdString(transformId), tx, ty, tz, rx, ry, rz, QString::fromStdString(parentId));
}

void InnerModel::updateTransformValuesS(std::string transformId, QVec v, std::string parentId)
{
		updateTransformValues(QString::fromStdString(transformId), v(0), v(1), v(2), v(3), v(4), v(5), QString::fromStdString(parentId));
}


void InnerModel::updateTransformValues(QString transformId, QVec v, QString parentId)
{
		updateTransformValues(transformId, v(0), v(1), v(2), v(3), v(4), v(5), parentId);
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

void InnerModel::updateJointValue(QString jointId, float angle, bool force)
{
	
	cleanupTables();

	InnerModelJoint *j = dynamic_cast<InnerModelJoint *>(hash[jointId]);
	if (j != NULL)
	{
		j->setAngle(angle, force);
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

InnerModelJoint *InnerModel::newJoint(QString id, InnerModelTransform *parent,float lx, float ly, float lz,float hx, float hy, float hz, float tx, float ty, float tz, float rx, float ry, float rz, float min, float max, uint32_t port,std::string axis, float home)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newJoint: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelJoint *newnode = new InnerModelJoint(id,lx,ly,lz,hx,hy,hz, tx, ty, tz, rx, ry, rz, min, max, port, axis, home, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelTouchSensor *InnerModel::newTouchSensor(QString id, InnerModelTransform *parent, QString stype, float nx, float ny, float nz, float min, float max, uint32_t port)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newTouchSensor: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelTouchSensor *newnode = new InnerModelTouchSensor(id, stype, nx, ny, nz, min, max, port, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelPrismaticJoint *InnerModel::newPrismaticJoint(QString id, InnerModelTransform *parent, float min, float max, float value, float offset, uint32_t port,std::string axis, float home)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newPrismaticJoint: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelPrismaticJoint *newnode = new InnerModelPrismaticJoint(id, min, max, value, offset, port, axis, home, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelDifferentialRobot *InnerModel::newDifferentialRobot(QString id, InnerModelTransform *parent, float tx, float ty, float tz, float rx, float ry, float rz, uint32_t port, float noise, bool collide)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newDifferentialRobot: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelDifferentialRobot *newnode = new InnerModelDifferentialRobot(id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelOmniRobot *InnerModel::newOmniRobot(QString id, InnerModelTransform *parent, float tx, float ty, float tz, float rx, float ry, float rz, uint32_t port, float noise, bool collide)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newOmniRobot: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelOmniRobot *newnode = new InnerModelOmniRobot(id, tx, ty, tz, rx, ry, rz, port, noise, collide, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelCamera *InnerModel::newCamera(QString id, InnerModelNode *parent, float width, float height, float focal)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newCamera: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelCamera *newnode = new InnerModelCamera(id, width, height, focal, this, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelRGBD *InnerModel::newRGBD(QString id, InnerModelNode *parent, float width, float height, float focal, float noise, uint32_t port, QString ifconfig)
{
	
	if (noise < 0)
	{
		QString error;
		error.sprintf("InnerModel::newRGBD: noise can't have negative values");
		throw error;
	}
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newRGBD: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelRGBD *newnode = new InnerModelRGBD(id, width, height, focal, noise, port, ifconfig, this, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelIMU *InnerModel::newIMU(QString id, InnerModelNode *parent, uint32_t port)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newIMU: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	// 	printf("newIMU id=%s  parentId=%s port=%d\n", id.toStdString().c_str(), parent->id.toStdString().c_str(), port);
	InnerModelIMU *newnode = new InnerModelIMU(id, port, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelLaser *InnerModel::newLaser(QString id, InnerModelNode *parent, uint32_t port, uint32_t min, uint32_t max, float angle, uint32_t measures, QString ifconfig)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newLaser: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelLaser *newnode = new InnerModelLaser(id, port, min, max, angle, measures, ifconfig, this, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelPlane *InnerModel::newPlane(QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx, float ny, float nz, float px, float py, float pz, bool collidable)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newPlane: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelPlane *newnode = new InnerModelPlane(id, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz, collidable, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelDisplay *InnerModel::newDisplay(QString id,uint32_t port, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, float nx, float ny, float nz, float px, float py, float pz, bool collidable)
{

	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newDisplay: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		printf("ERROR: %s\n", error.toStdString().c_str());
		throw error;
	}
	InnerModelDisplay *newnode = new InnerModelDisplay(id, port, texture, width, height, depth, repeat, nx, ny, nz, px, py, pz, collidable, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelMesh *InnerModel::newMesh(QString id, InnerModelNode *parent, QString path, float scalex, float scaley, float scalez, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newMesh: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelMesh *newnode = new InnerModelMesh(id, path, scalex, scaley, scalez, (InnerModelMesh::RenderingModes)render, tx, ty, tz, rx, ry, rz, collidable, parent);
	hash[id] = newnode;
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelMesh *InnerModel::newMesh(QString id, InnerModelNode *parent, QString path, float scale, int render, float tx, float ty, float tz, float rx, float ry, float rz, bool collidable)
{
	
	return newMesh(id,parent,path,scale,scale,scale,render,tx,ty,tz,rx,ry,rz, collidable);
}

InnerModelPointCloud *InnerModel::newPointCloud(QString id, InnerModelNode *parent)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newPointCloud: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelPointCloud *newnode = new InnerModelPointCloud(id, parent);
	hash[id] = newnode;
	//printf("Inserted point cloud %s ptr(%p), on node %s\n", id.toStdString().c_str(), newnode, parent->id.toStdString().c_str());
// 	parent->addChild(newnode);
	return newnode;
}

InnerModelTransform *InnerModel::newTransform(QString id, QString engine, InnerModelNode *parent, float tx, float ty, float tz, float rx, float ry, float rz, float mass)
{
	
	if (hash.contains(id))
	{
		QString error;
		error.sprintf("InnerModel::newTransform: Error: Trying to insert a node with an already-existing key: %s\n", id.toStdString().c_str());
		throw error;
	}
	InnerModelTransform *newnode = new InnerModelTransform(id, engine, tx, ty, tz, rx, ry, rz, mass, parent);
	hash[id] = newnode;
// 	std::cout << (void *)newnode << "  " << (uint64_t)newnode << std::endl;
// 	parent->addChild(newnode);
	return newnode;
}


//////////////////////////////////////////////////////////////////////////////////////////
/// Information retrieval methods
//////////////////////////////////////////////////////////////////////////////////////////

QVec InnerModel::transform(const QString &destId, const QVec &initVec, const QString &origId)
{
	
	if (initVec.size()==3)
	{
		return (getTransformationMatrix(destId, origId) * initVec.toHomogeneousCoordinates()).fromHomogeneousCoordinates();
	}
	else if (initVec.size()==6)
	{
		const QMat M = getTransformationMatrix(destId, origId);
		const QVec a = (M * initVec.subVector(0,2).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
		const Rot3D R(initVec(3), initVec(4), initVec(5));

		const QVec b = (M.getSubmatrix(0,2,0,2)*R).extractAnglesR_min();
		QVec ret(6);
		ret(0) = a(0);
		ret(1) = a(1);
		ret(2) = a(2);
		ret(3) = b(0);
		ret(4) = b(1);
		ret(5) = b(2);
		return ret;
	}
	else
	{
		throw InnerModelException("InnerModel::transform was called with an unsupported vector size.");
	}
}

QVec InnerModel::transformS(const std::string & destId, const QVec &origVec, const std::string & origId)
{
		return transform(QString::fromStdString(destId), origVec, QString::fromStdString(origId));
}

QVec InnerModel::rotationAngles(const QString & destId, const QString & origId)
{
	return getTransformationMatrix(destId, origId).extractAnglesR();
}


/// Matrix transformation retrieval methods
RTMat InnerModel::getTransformationMatrix(const QString &to, const QString &from)
{
	RTMat ret;

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

RTMat InnerModel::getTransformationMatrixS(const std::string &destId, const std::string &origId)
{
	return getTransformationMatrix(QString::fromStdString(destId), QString::fromStdString(origId));
}


QMat InnerModel::getRotationMatrixTo(const QString &to, const QString &from)
{
	QMat rret = QMat::identity(3);

	

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
			// 			error.sprintf("InnerModel::setLists: It wouldn't be here!!!!");
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
			// 			error.sprintf("InnerModel::setLists: It wouldn't be here!!!!");
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

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------


bool InnerModel::collidable(const QString &a)
{
	
	InnerModelNode *node;
	try
	{
		node = hash[a];
	}
	catch(...)
	{
		qDebug() <<__FUNCTION__ << "No node" << a;
	}
	if (node)
	{
		if (node->collidable)
			return true;
		return false;
	}
	return false;
}

bool InnerModel::collide(const QString &a, const QString &b)
{
	
#if FCL_SUPPORT==1
	InnerModelNode *n1 = getNode(a);
	if (not n1) throw 1;
	//qDebug() << "collide " << a << b << n1->id;
	QMat r1q = getRotationMatrixTo("root", a);
	fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
	QVec t1v = getTranslationVectorTo("root", a);
	fcl::Vec3f T1( t1v(0), t1v(1), t1v(2) );
	n1->collisionObject->setTransform(R1, T1);

	InnerModelNode *n2 = getNode(b);
	if (not n1) throw 2;
	QMat r2q = getRotationMatrixTo("root", b);
	fcl::Matrix3f R2( r2q(0,0), r2q(0,1), r2q(0,2), r2q(1,0), r2q(1,1), r2q(1,2), r2q(2,0), r2q(2,1), r2q(2,2) );
	QVec t2v = getTranslationVectorTo("root", b);
	fcl::Vec3f T2( t2v(0), t2v(1), t2v(2) );
	n2->collisionObject->setTransform(R2, T2);

	fcl::CollisionRequest request;
	fcl::CollisionResult result;

	n1->collisionObject->computeAABB();
 	fcl::AABB a1 = n1->collisionObject->getAABB();
 	fcl::Vec3f v1 = a1.center();

	n2->collisionObject->computeAABB();
 	fcl::AABB a2 = n2->collisionObject->getAABB();
 	fcl::Vec3f v2 = a2.center();

 	//qDebug()<< a;
 	//printf("- (%f,  %f,  %f) --- (%f,  %f,  %f) [%f , %f , %f]  <<%f %d>>\n", v1[0], v1[1], v1[2], (v1-v2)[0], (v1-v2)[1], (v1-v2)[2], a1.width(), a1.height(), a1.depth(), a1.distance(a2), a1.overlap(a2));
 	//qDebug()<< b;
 	//printf("- (%f,  %f,  %f) --- (%f,  %f,  %f) [%f , %f , %f]  <<%f %d>>\n", v2[0], v2[1], v2[2], (v1-v2)[0], (v1-v2)[1], (v1-v2)[2], a2.width(), a2.height(), a2.depth(), a1.distance(a2), a1.overlap(a2));

	// NOTE: Un poco de documentacion nunca esta mal, sabeis --> http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/namespacefcl.html
	// std::size_t 	collide (const CollisionObject *o1, const CollisionObject *o2, const CollisionRequest &request, CollisionResult &result)
	fcl::collide(                  n1->collisionObject,       n2->collisionObject,                         request,                  result);
	//qDebug() << result.isCollision();
	///qDebug() << "--------------";
	// return binary collision result --> http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/structfcl_1_1CollisionResult.html#ed599cb31600ec6d0585d9adb4cde946
	// True if There are collisions, and false if there arent collisions.
	return result.isCollision();
#else
	QString error;
	error.sprintf("InnerModel was not compiled with collision support");
	throw error;
	return false;
#endif
}


/**
 * @brief ...
 *
 * @param a ...
 * @param obj ...
 * @return bool
 */
#if FCL_SUPPORT==1
bool InnerModel::collide(const QString &a, const fcl::CollisionObject *obj)
{
	
	InnerModelNode *n1 = getNode(a);
	if (not n1) throw 1;
	QMat r1q = getRotationMatrixTo("root", a);
	fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
	QVec t1v = getTranslationVectorTo("root", a);
	fcl::Vec3f T1( t1v(0), t1v(1), t1v(2) );
	n1->collisionObject->setTransform(R1, T1);

	fcl::CollisionRequest request;
	fcl::CollisionResult result;

	fcl::collide(n1->collisionObject, obj, request, result);

	return result.isCollision();
}
#endif

QMat InnerModel::jacobian(QStringList &listaJoints, const QVec &motores, const QString &endEffector)
{
	// La lista de motores define una secuencia contigua de joints, desde la base hasta el extremo.
	// Inicializamos las filas del Jacobiano al tamaño del punto objetivo que tiene 6 ELEMENTOS [tx, ty, tz, rx, ry, rz]
	// y las columnas al número de motores (Joints): 6 filas por n columnas. También inicializamos un vector de ceros

	
	QMat jacob(6, listaJoints.size(), 0.f);  //6 output variables
	QVec zero = QVec::zeros(3);
	int j=0; //índice de columnas de la matriz: MOTORES

	foreach(QString linkName, listaJoints)
	{
		if(motores[j] == 0)
		{
			QString frameBase = listaJoints.last();

			// TRASLACIONES: con respecto al último NO traslada
			QVec axisTip = getJoint(linkName)->unitaryAxis(); 		//vector de ejes unitarios
			axisTip = transform(frameBase, axisTip, linkName);
			QVec axisBase = transform(frameBase, zero, linkName);
			QVec axis = axisBase - axisTip;
			QVec toEffector = (axisBase - transform(frameBase, zero, endEffector) );
			QVec res = toEffector.crossProduct(axis);

			jacob(0,j) = res.x();
			jacob(1,j) = res.y();
			jacob(2,j) = res.z();

			// ROTACIONES
			QVec axisTip2 = getJoint(linkName)->unitaryAxis(); 		//vector de ejes unitarios en el que gira
			axisTip2 = transform(frameBase, axisTip2, linkName); 		//vector de giro pasado al hombro.
			QVec axisBase2 = transform(frameBase, zero, linkName); 	//motor al hombro
			QVec axis2 = axisBase2 - axisTip2; 				//vector desde el eje de giro en el sist. hombro, hasta la punta del eje de giro en el sist. hombro.

			jacob(3,j) = axis2.x();
			jacob(4,j) = axis2.y();
			jacob(5,j) = axis2.z();
		}
		j++;
	}
	return jacob;
}

QString InnerModel::getParentIdentifier(QString id)
{
	
	InnerModelNode *n = getNode(id);
	if (n)
	{
		if (n->parent)
			return n->parent->id;
		else
			return QString("");
	}
	return QString("");
}

std::string InnerModel::getParentIdentifierS(std::string id)
{
	return getParentIdentifier(QString::fromStdString(id)).toStdString();
}



float InnerModel::distance(const QString &a, const QString &b)
{
#if FCL_SUPPORT==1
	InnerModelNode *n1 = getNode(a);
	if (not n1) throw 1;
	QMat r1q = getRotationMatrixTo("root", a);
	fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
	QVec t1v = getTranslationVectorTo("root", a);
	fcl::Vec3f T1( t1v(0), t1v(1), t1v(2) );
	n1->collisionObject->setTransform(R1, T1);

	InnerModelNode *n2 = getNode(b);
	if (not n1) throw 2;
	QMat r2q = getRotationMatrixTo("root", b);
	fcl::Matrix3f R2( r2q(0,0), r2q(0,1), r2q(0,2), r2q(1,0), r2q(1,1), r2q(1,2), r2q(2,0), r2q(2,1), r2q(2,2) );
	QVec t2v = getTranslationVectorTo("root", b);
	fcl::Vec3f T2( t2v(0), t2v(1), t2v(2) );
	n2->collisionObject->setTransform(R2, T2);

	fcl::DistanceResult result;
	fcl::DistanceRequest request;

	n1->collisionObject->computeAABB();
	n2->collisionObject->computeAABB();

	fcl::distance(n1->collisionObject, n2->collisionObject, request, result);
	return result.min_distance;
#else
	QString error;
	error.sprintf("InnerModel was not compiled with collision support");
	throw error;
	return -1;
#endif
}

// QVec InnerModel::project(QString reference, QVec origVec, QString cameraId)
// {
// 	origVec = transform(cameraId, origVec, reference);
//
// 	QVec pc;
// 	InnerModelCamera *camera=NULL;
//
// 	camera = dynamic_cast<InnerModelCamera *>(hash[cameraId]);
// 	if (not camera)
// 	{
// 		QString error;
// 		error.sprintf("No such %s camera", qPrintable(cameraId));
// 		throw error;
// 	}
//
// 	pc = camera->camera.project(origVec);
//
// 	return QVec::vec3(pc(0), pc(1), origVec.norm2());
// }


// QVec InnerModel::project(const QString &cameraId, const QVec &origVec)
// {
// 	QVec pc;
// 	InnerModelCamera *camera=NULL;
//
// 	camera = dynamic_cast<InnerModelCamera *>(hash[cameraId]);
// 	if (not camera)
// 	{
// 		QString error;
// 		error.sprintf("No such %s camera", qPrintable(cameraId));
// 		throw error;
// 	}
//
// 	pc = camera->camera.project(origVec);
//
// 	return QVec::vec3(pc(0), pc(1), origVec.norm2());
// }
//
//
//
// /**
//  * \brief Retro-projection function, defines a line in the camera reference system which can be parametrized by the depth s with the expression:
//  * p = s*[ (u-u0) / alfaU ; (v-v0) / alfaV ; 1] being alfaU and alfaV the horizontal and vertical focals of the camera (in pixels)
//  * p has value 1 in the z axis, the one going out the camera.
//  * @param cameraId name of camara to be used as known in innermodel tree
//  * @param coord  point in image coordinates
//  * @return a line en camera reference system that can be parametrized by the depth s
//  */
// QVec InnerModel::backProject( const QString &cameraId, const QVec &	coord) //const
// {
// 	if(hash.contains(cameraId))
// 	{
// 		QVec p = static_cast<InnerModelCamera *>(hash[cameraId])->camera.getRayHomogeneous(coord);
// 		return p;
// 	}
// 	return QVec();
// }
//
//
//
// void InnerModel::imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS)
// {
// 	QVec ray = backProject(cameraId, coord);
//
// 	QVec finalRay = getRotationMatrixTo(anglesRefS, cameraId)*ray;
//
// 	pan = atan2(finalRay(0), finalRay(2));
// 	tilt = atan2(finalRay(1), finalRay(2));
//
// }
//
//
//
// QVec InnerModel::anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS)
// {
// 	QVec p(3), ray(3);
//
// 	p(0) = tan(pan);
// 	p(1) = tan(tilt);
// 	p(2) = 1;
//
// 	ray = getRotationMatrixTo(cameraId, anglesRefS) * p;
// 	ray(0)=ray(0)/ray(2);
// 	ray(1)=ray(1)/ray(2);
// 	ray(2)=1;
//
// 	return project(cameraId, ray, cameraId);
//
// }
//
//
//
// QVec InnerModel::imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString reference)
// {
// 	//We obtain a 3D line (a,b,1) in camera reference system that can be parametrized in depth to obtain a point at "depth" from the camera.
// 	QVec p = backProject( cameraId, coord ) * depth;
// 	//Now we transform it to requested node of the robot.
// 	if(p.size()>0)
// 		return transform(reference, p, cameraId);
// 	return p;
// }
//
//
//
// QVec InnerModel::projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist)
// {
// 	QMat mSystem(3,3);
// 	QVec tIndep(3);
// 	QVec pCam(3);
// 	QVec res(3);
// 	float dxz, dyz;
//
// 	pCam(0) = -coord(0)+getCameraWidth(cameraId)/2;
// 	pCam(1) = -(coord(1)-getCameraHeight(cameraId)/2);
// 	pCam(2) = getCameraFocal(cameraId);
// 	QVec pDest = transform(to, pCam, cameraId);
// 	QVec pCent = transform(to, QVec::vec3(0,0,0), cameraId);
// 	QVec direc = pDest-pCent;
// 	dxz = direc(0)/direc(2);
// 	dyz = direc(1)/direc(2);
//
// 	res(2) = dist + vPlane(0)*(dxz*pCent(2)-pCent(0)) + vPlane(1)*(dyz*pCent(2)-pCent(1));
// 	res(2) = res(2)/(vPlane(0)*dxz+vPlane(1)*dyz+vPlane(2));
// 	res(0)=dxz*(res(2)-pCent(2))+pCent(0);
// 	res(1)=dyz*(res(2)-pCent(2))+pCent(1);
//
// 	/*	res.print("res");
// 	 *
// 	 *	mSystem(0,0) = vPlane(0);         mSystem(0,1) = vPlane(1);         mSystem(0,2) = vPlane(2);
// 	 *	mSystem(1,0) = 0;                 mSystem(1,1) = pCent(2)-pDest(2); mSystem(1,2) = pDest(1)-pCent(1);
// 	 *	mSystem(2,0) = pDest(2)-pCent(2); mSystem(2,1) = 0;                 mSystem(2,2) = pCent(0)-pDest(0);
// 	 *	tIndep(0) = dist;
// 	 *	tIndep(1) = pCent(2)*(pDest(1)-pCent(1))+pCent(1)*(pCent(2)-pDest(2));
// 	 *	tIndep(2) = pCent(0)*(pDest(2)-pCent(2))+pCent(2)*(pCent(0)-pDest(0));
// 	 *
// 	 * 	return (mSystem.invert())*tIndep;*/
// 	return res;
// }
//
//
//
// //
// // bool InnerModel::check3DPointInsideFrustrum(QString cameraId, QVec coor)
// // {
// // }
//
// /**
//  * \brief Returns a 3D vector (A,B,C) containing the horizon line for the specified camera+plane in the form 'Ax + By + C = 0'.
//  *
//  * <p>
//  * Returns a 3D vector (A,B,C) containing the horizon line in the form Ax + By + C = 0. For general lines, it will also work as 'y = Ax + C' (not for vertical lines, which are a very rare case).
//  * You can check B to know if the returned vector is a regular line:
//  * </p>
//  * <p>
//  * QVec horizon = innerModel->horizonLine("floor", "mycamera", );
//  * <br>
//  * if (horizon(1) == 0) printf("Vertical horizon.\n");
//  * <br>
//  * else printf("Regular horizon.\n");
//  * </p>
//  */
// QVec InnerModel::horizonLine(QString planeId, QString cameraId, float heightOffset)
// {
// 	
// 	// 	printf("-------------------------------------- cam:%s plane:%s\n", qPrintable(cameraId), qPrintable(planeId));
// 	// Get camera and plane pointers
// 	InnerModelPlane *plane = getPlane(planeId);
// 	InnerModelCamera *camera = getCamera(cameraId);
// 	// Transform rotate plane normal vector to camera reference system
// 	QMat rtm = getRotationMatrixTo(cameraId, planeId);
// 	QVec vec = QVec::vec3(plane->normal(0), plane->normal(1), plane->normal(2));
// 	QVec normal = rtm*vec;
// 	if (normal(1) <= 0.0000002) throw false;
//
// 	// Create two points
// 	QVec p1=QVec::vec3(0., 0., 0.), p2=QVec::vec3(0., 0., 0.);
// 	// Move both points forward
// 	p1(2) = p2(2) =  1000.;
// 	if (normal(1) > 0.0000001) p1(1) = p2(1) = p1(2)*normal(2)/normal(1);
// 	// Move points left/right-wards
// 	if (normal(1) > 0.0000001) p1(1) -=  200.*normal(0)/normal(1);
// 	p1(0) =  200.;
// 	if (normal(1) > 0.0000001) p2(1) -= -200.*normal(0)/normal(1);
// 	p2(0) = -200.;
// 	// Project points
// 	p1 = project(cameraId, p1, cameraId);
// 	p2 = project(cameraId, p2, cameraId);
// 	// Compute image line
// 	double dx=p2(0)-p1(0);
// 	double dy=p2(1)-p1(1);
//
// 	if (abs(dx) <= 1)
// 	{
// 		if (abs(dy) <= 1)
// 		{
// 			QString error;
// 			error.sprintf("Degenerated camera");
// 			throw error;
// 		}
// 		return QVec::vec3(-1, 0, p1(0));
// 	}
// 	else
// 	{
// 		return QVec::vec3(dy/dx, -1, camera->camera.getHeight()-(p1(1)-(dy*p1(0)/dx))+heightOffset);
// 	}
// }
//
//
//

//
// QMat InnerModel::getHomographyMatrix(QString virtualCamera, QString plane, QString sourceCamera)
// {
// 	
//
// 	QVec planeN = getPlane(plane)->normal;
// 	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
// 	QVec planePoint = transform(sourceCamera, getPlane(plane)->point, plane);
//
// 	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
// 	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
// 	QMat n  = QMat(planeN);
// 	QMat K1 = getCamera(sourceCamera)->camera;
// 	QMat K2 = getCamera(virtualCamera)->camera;
//
// 	double d = -(planePoint*planeN);
// 	QMat H = K2 * ( R - ((t*n.transpose()) / d) ) * K1.invert();
// 	return H;
// }
//
//
//
// QMat InnerModel::getAffineHomographyMatrix(QString virtualCamera, QString plane, QString sourceCamera)
// {
// 	
//
// 	QVec planeN = getPlane(plane)->normal;
// 	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
// 	QVec planePoint = transform(sourceCamera, getPlane(plane)->point, plane);
//
// 	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
// 	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
// 	QMat n  = QMat(planeN);
// 	QMat K1 = getCamera(sourceCamera)->camera;
//
// 	double d = -(planePoint*planeN);
// 	QMat H = ( R - ((t*n.transpose()) / d) ) * K1.invert();
// 	for (int r=0;r<2;r++)
// 		for (int c=0;c<3;c++)
// 			H(r,c) = H(r,c) * 1000.;
// 		return H;
// }
//
//
//
// QMat InnerModel::getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera)
// {
// 	
// 	QVec planeN = getPlane(plane)->normal;
// 	planeN = getRotationMatrixTo(sourceCamera, plane)*planeN;
// 	QVec planePoint = transform(sourceCamera, getPlane(plane)->point, plane);
//
// 	QMat R  = getRotationMatrixTo(virtualCamera, sourceCamera);
// 	QMat t  = transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
// 	QMat n  = QMat(planeN);
// 	QMat K1 = getCamera(sourceCamera)->camera;
//
// 	double d = -(planePoint*planeN);
// 	QMat H = ( R - ((t*n.transpose()) / d) ) * K1.invert();
// 	QMat HFinal(4,3);
// 	HFinal.inject(H, 0, 0);
// 	HFinal = HFinal*(1000*1000);
// 	HFinal(3,0)=1000*H(2,0);
// 	HFinal(3,1)=1000*H(2,1);
// 	HFinal(3,2)=1000*H(2,2);
// 	return HFinal;
// }
//
//




// float InnerModel::getCameraFocal(const QString & cameraId) const
// {
// 	
// 	InnerModelCamera *cam = dynamic_cast<InnerModelCamera *>(getNode(cameraId));
// 	if (not cam)
// 	{
// 		QString error;
// 		       printf("InnerModel::getCameraFocal, no such camera %s\n", cameraId.toStdString().c_str());
// 		error.sprintf("InnerModel::getCameraFocal, no such camera %s\n", cameraId.toStdString().c_str());
// 		throw error;
// 	}
// 	return cam->getFocal();
// }
//
//
//
// int InnerModel::getCameraWidth(QString cameraId)
// {
// 	
// 	return getCamera(cameraId)->getWidth();
// }
//
//
//
// int InnerModel::getCameraHeight(const QString & cameraId) const
// {
// 	
// 	return static_cast<InnerModelCamera *>(getNode(cameraId))->getHeight();
// }
//
//
//
// int InnerModel::getCameraSize(const QString & cameraId) const
// {
// 	
// 	return static_cast<InnerModelCamera *>(getNode(cameraId))->getSize();
// }
//

// /**
//  * \brief Local laser measure of range r and angle alfa is converted to Any RS
//  * @param r range measure
//  * @param alfa angle measure
//  * @return 3-vector of x,y,z coordinates un WRS
//  */
// QVec InnerModel::laserTo(const QString &dest, const QString & laserId , float r, float alpha)
// {
// 	
// 	QVec p(3);
// 	p(0) = r * sin(alpha);
// 	p(1) = 0;
// 	p(2) = r * cos(alpha);
// 	return transform(dest, p, laserId);
// }

// QVec InnerModel::compute3DPointFromImageCoords(const QString &firstCamera, const QVec &left, const QString &secondCamera, const QVec &right, const QString &refSystem)
// {
// 	
// 	QVec pI(3), pD(3), n(3), ray(3), T(3), TI(3), TD(3), pR(0), abc(3);
// 	QMat A(3,3);
//
// 	ray = backProject(firstCamera, left);
// 	pI = getRotationMatrixTo(refSystem, firstCamera)*ray;
// 	pI(0)=pI(0)/pI(2);
// 	pI(1)=pI(1)/pI(2);
// 	pI(2)=1.;
//
// 	ray = backProject(secondCamera, right);
// 	pD = getRotationMatrixTo(refSystem, secondCamera)*ray;
// 	pD(0)=pD(0)/pD(2);
// 	pD(1)=pD(1)/pD(2);
// 	pD(2)=1.;
//
// 	n = pI ^ pD;
//
// 	A(0,0)=pI(0);  A(0,1)=-pD(0);  A(0,2)=n(0);
// 	A(1,0)=pI(1);  A(1,1)=-pD(1);  A(1,2)=n(1);
// 	A(2,0)=pI(2);  A(2,1)=-pD(2);  A(2,2)=n(2);
//
// 	TI = getTranslationVectorTo(refSystem, firstCamera).fromHomogeneousCoordinates();
// 	TD = getTranslationVectorTo(refSystem, secondCamera).fromHomogeneousCoordinates();
// 	T = TD - TI ;
//
// 	abc = (A.invert())*T;
//
// 	pR = (pI*abc(0));
// 	pR = pR + TI;
// 	pR = (n*(abc(2)/2)) + pR;
//
// 	return pR;
// }
//
// QVec InnerModel::compute3DPointFromImageAngles(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem)
// {
// 	
// 	QVec pI(3), pD(3), n(3), ray(3), T(3), TI(3), TD(3), pR(0), abc(3);
// 	QMat A(3,3);
//
// 	ray(0) = tan(left(0));
// 	ray(1) = tan(left(1));
// 	ray(2) = 1.;
// 	pI = ray;//getRotationMatrixTo(refSystem, firstCamera)*ray;
//
// 	pI(0)=pI(0)/pI(2);
// 	pI(1)=pI(1)/pI(2);
// 	pI(2)=1.;
//
// 	ray(0) = tan(right(0));
// 	ray(1) = tan(right(1));
// 	ray(2) = 1.;
// 	pD = ray;//getRotationMatrixTo(refSystem, secondCamera)*ray;
//
// 	pD(0)=pD(0)/pD(2);
// 	pD(1)=pD(1)/pD(2);
// 	pD(2)=1.;
//
// 	n = pI ^ pD;
//
// 	A(0,0)=pI(0);  A(0,1)=-pD(0);  A(0,2)=n(0);
// 	A(1,0)=pI(1);  A(1,1)=-pD(1);  A(1,2)=n(1);
// 	A(2,0)=pI(2);  A(2,1)=-pD(2);  A(2,2)=n(2);
//
// 	TI = getTranslationVectorTo(refSystem, firstCamera).fromHomogeneousCoordinates();
// 	TD = getTranslationVectorTo(refSystem, secondCamera).fromHomogeneousCoordinates();
// 	T = TD - TI;
//
// 	abc = (A.invert())*T;
//
// 	pR = (pI*abc(0));
// 	pR = pR + TI;
// 	pR = (n*(abc(2)/2)) + pR;
//
// 	return pR;
// }
