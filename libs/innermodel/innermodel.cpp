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

#include <osg/io_utils>
#include <osgDB/ReadFile>
#include <osg/Geode>

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
	collidable = false;
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif

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
	if (child->parent != this and child->parent != NULL)
	{
// 		printf("InnerModelNode::addChild this is weird\n");
	}

// 	std::cout << "addchild_p: "<< (void *)this << "  " << (void *)child << std::endl;
// 	std::cout << "addchild_u: "<< (uint64_t)this << "  " << (uint64_t)child << std::endl;
	if (not children.contains(child))
	{
		children.append(child);
	}
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



void InnerModelNode::updateChildren()
{
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
	mutex = new QMutex(QMutex::Recursive);
	// Set Root node
	InnerModelTransform *root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	root->parent = NULL;
	setRoot(root);
	hash["root"] = root;

	// How to use:
	//   InnerModelTransform *tr = innerModel->newTransform("name", parent, rx, ry, rz, px, py, pz);
	//   parent->addChild(tr);
}



InnerModel::InnerModel(const InnerModel &original)
{
	mutex = new QMutex(QMutex::Recursive);
	root = new InnerModelTransform("root", "static", 0, 0, 0, 0, 0, 0, 0);
	setRoot(root);
	hash["root"] = root;

	QList<InnerModelNode *>::iterator i;
	for (i=original.root->children.begin(); i!=original.root->children.end(); i++)
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
	{
		inner->root->addChild((*i)->copyNode(inner->hash, inner->root));
	}
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

// void InnerModel::getSubTreeN(InnerModelNode *orig, InnerModelNode *ret)
// {
// 	//ASSERTS
//
// 	QList<InnerModelNode *>::iterator i;
// 	for (i=orig->children.begin(); i!=orig->children.end(); i++)
// 	{
// 		ret->addChild((*i)->copyNode(hash, root));
// 	}
// }

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
			{
			}
				//qDebug() << "?????";
		}
		//always update
		aux->update(tx,ty,tz,rx,ry,rz);
	}
	else if (hash[transformId] == NULL)
		qDebug() << "There is no such" << transformId << "node";
	else
	{
		//qDebug() << "?????";
	}
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
	InnerModelCamera *newnode = new InnerModelCamera(id, width, height, focal, parent);
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
	InnerModelRGBD *newnode = new InnerModelRGBD(id, width, height, focal, noise, port, ifconfig, parent);
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
	// 	printf("newLaser id=%s  parentId=%s port=%d min=%d max=%d angle=%f measures=%d\n", id.toStdString().c_str(), parent->id.toStdString().c_str(), port, min, max, angle, measures);
	InnerModelLaser *newnode = new InnerModelLaser(id, port, min, max, angle, measures, ifconfig, parent);
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



InnerModelTransform *InnerModel::getTransform(const QString &id)
{
	InnerModelTransform *tr = dynamic_cast<InnerModelTransform *>(hash[id]);
	if (not tr)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such transform %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a transform", id.toStdString().c_str());
		throw error;
	}
	return tr;
}



InnerModelJoint *InnerModel::getJoint(const QString &id)
{
	InnerModelJoint *tr = dynamic_cast<InnerModelJoint *>(hash[id]);
	if (not tr)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such joint %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a joint", id.toStdString().c_str());
		throw error;
	}
	return tr;
}



InnerModelTouchSensor *InnerModel::getTouchSensor(const QString &id)
{
	InnerModelTouchSensor *tr = dynamic_cast<InnerModelTouchSensor *>(hash[id]);
	if (not tr)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such touch sensor %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a touch sensor", id.toStdString().c_str());
		throw error;
	}
	return tr;
}



InnerModelPrismaticJoint *InnerModel::getPrismaticJoint(const QString &id)
{
	InnerModelPrismaticJoint *tr = dynamic_cast<InnerModelPrismaticJoint *>(hash[id]);
	if (not tr)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such joint %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a prismatic joint", id.toStdString().c_str());
		throw error;
	}
	return tr;
}



InnerModelCamera *InnerModel::getCamera(const QString id)
{
	InnerModelCamera *camera = dynamic_cast<InnerModelCamera *>(hash[id]);
	if (not camera)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such camera %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a camera", id.toStdString().c_str());
		throw error;
	}
	return camera;
}



InnerModelRGBD *InnerModel::getRGBD(const QString id)
{
	InnerModelRGBD *camera = dynamic_cast<InnerModelRGBD *>(hash[id]);
	if (not camera)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such camera %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a camera", id.toStdString().c_str());
		throw error;
	}
	return camera;
}



InnerModelIMU *InnerModel::getIMU(const QString id)
{
	InnerModelIMU *imu = dynamic_cast<InnerModelIMU *>(hash[id]);
	if (not imu)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such innertial unit %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be an innertial unit", id.toStdString().c_str());
		throw error;
	}
	return imu;
}



InnerModelLaser *InnerModel::getLaser(const QString id)
{
	InnerModelLaser *laser = dynamic_cast<InnerModelLaser *>(hash[id]);
	if (not laser)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such laser %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be an laser", id.toStdString().c_str());
		throw error;
	}
	return laser;
}



InnerModelPlane *InnerModel::getPlane(const QString &id)
{
	InnerModelPlane *plane = dynamic_cast<InnerModelPlane *>(hash[id]);
	if (not plane)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such plane %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a plane", id.toStdString().c_str());
		throw error;
	}
	return plane;
}



InnerModelMesh *InnerModel::getMesh(const QString &id)
{
	InnerModelMesh *mesh = dynamic_cast<InnerModelMesh *>(hash[id]);
	if (not mesh)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such mesh %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a mesh", id.toStdString().c_str());
		throw error;
	}
	return mesh;
}



InnerModelPointCloud *InnerModel::getPointCloud(const QString &id)
{
	InnerModelPointCloud *pointcloud = dynamic_cast<InnerModelPointCloud *>(hash[id]);
	if (not pointcloud)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such pointcloud %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a pointcloud", id.toStdString().c_str());
		throw error;
	}
	return pointcloud;
}



InnerModelDifferentialRobot *InnerModel::getDifferentialRobot(const QString &id)
{
	InnerModelDifferentialRobot *diff = dynamic_cast<InnerModelDifferentialRobot *>(hash[id]);
	if (not diff)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such differential robot %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a differential robot", id.toStdString().c_str());
		throw error;
	}
	return diff;
}

InnerModelOmniRobot *InnerModel::getOmniRobot(const QString &id)
{
	InnerModelOmniRobot *diff = dynamic_cast<InnerModelOmniRobot *>(hash[id]);
	if (not diff)
	{
		QString error;
		if (not hash[id])
			error.sprintf("No such omni robot %s", id.toStdString().c_str());
		else
			error.sprintf("%s doesn't seem to be a omni robot", id.toStdString().c_str());
		throw error;
	}
	return diff;
}





QVec InnerModel::compute3DPointFromImageCoords(const QString &firstCamera, const QVec &left, const QString &secondCamera, const QVec &right, const QString &refSystem)
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

QVec InnerModel::rotationAngles(const QString & destId, const QString & origId)
{
	return getTransformationMatrix(destId, origId).extractAnglesR();
}

QVec InnerModel::project(QString reference, QVec origVec, QString cameraId)
{
	origVec = transform(cameraId, origVec, reference);

	QVec pc;
	InnerModelCamera *camera=NULL;

	camera = dynamic_cast<InnerModelCamera *>(hash[cameraId]);
	if (not camera)
	{
		QString error;
		error.sprintf("No such %s camera", qPrintable(cameraId));
		throw error;
	}

	pc = camera->camera.project(origVec);

	return QVec::vec3(pc(0), pc(1), origVec.norm2());
}


QVec InnerModel::project(const QString &cameraId, const QVec &origVec)
{
	QVec pc;
	InnerModelCamera *camera=NULL;

	camera = dynamic_cast<InnerModelCamera *>(hash[cameraId]);
	if (not camera)
	{
		QString error;
		error.sprintf("No such %s camera", qPrintable(cameraId));
		throw error;
	}

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
		if (abs(dy) <= 1)
		{
			QString error;
			error.sprintf("Degenerated camera");
			throw error;
		}
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

RTMat InnerModel::getTransformationMatrixS(const std::string &destId, const std::string &origId)
{
	return getTransformationMatrix(QString::fromStdString(destId), QString::fromStdString(origId));
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



/// Robex Base specific getters
float InnerModel::getCameraFocal(const QString & cameraId) const
{
	InnerModelCamera *cam = dynamic_cast<InnerModelCamera *>(getNode(cameraId));
	if (not cam)
	{
		QString error;
		       printf("InnerModel::getCameraFocal, no such camera %s\n", cameraId.toStdString().c_str());
		error.sprintf("InnerModel::getCameraFocal, no such camera %s\n", cameraId.toStdString().c_str());
		throw error;
	}
	return cam->getFocal();
}



int InnerModel::getCameraWidth(QString cameraId)
{
	return getCamera(cameraId)->getWidth();
}



int InnerModel::getCameraHeight(const QString & cameraId) const
{
	return static_cast<InnerModelCamera *>(getNode(cameraId))->getHeight();
}



int InnerModel::getCameraSize(const QString & cameraId) const
{
	return static_cast<InnerModelCamera *>(getNode(cameraId))->getSize();
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
	p(1) = 0;
	p(2) = r * cos(alpha);
	return transform(dest, p, laserId);
}


// ------------------------------------------------------------------------------------------------
// InnerModelTransform
// ------------------------------------------------------------------------------------------------

InnerModelTransform::InnerModelTransform(QString id_, QString engine_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float mass_, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
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
			out << "<rotation id=\"" << id << "\" rx=\""<< QString::number(backrX, 'g', 10) <<"\" ry=\""<< QString::number(backrY, 'g', 10) <<"\" rz=\""<< QString::number(backrZ, 'g', 10) <<"\">\n";
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



/**
 * @brief Updates the internal values of the node from the values passed in the parameters
 *
 * @param tx_ X Translation
 * @param ty_ Y Translation
 * @param tz_ Z Translation
 * @param rx_ RX Rotation
 * @param ry_ RY Rotation
 * @param rz_ RZ Rotation
 * @return void
 */
void InnerModelTransform::update(float tx_, float ty_, float tz_, float rx_, float ry_, float rz_)
{
	backrX = rx_; backrY = ry_; backrZ = rz_;
	backtX = tx_; backtY = ty_; backtZ = tz_;
	set(backrX, backrY, backrZ, backtX, backtY, backtZ);
	fixed = true;
}


InnerModelNode * InnerModelTransform::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelTransform *ret = new InnerModelTransform(id, engine, backtX, backtY, backtZ, backrX, backrY, backrZ, mass, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}


// ------------------------------------------------------------------------------------------------
// InnerModelJoint
// ------------------------------------------------------------------------------------------------

InnerModelJoint::InnerModelJoint(QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, float min_, float max_, uint32_t port_, std::string axis_, float home_, InnerModelTransform *parent_) : InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
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
		QString error;
		error.sprintf("internal error, no such axis %s\n", axis.c_str());
		throw error;
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
	QList<InnerModelNode*>::iterator c;
	//<joint id="head_yaw_joint" port="10067" axis="z" home="0" min="-1" max="1">
	for (int i=0; i<tabs; i++) out << "\t";	
	out << "<joint id=\"" << id << "\" port=\"" << port << "\" axis=\"" <<QString::fromStdString( axis)<<"\" home=\""<< QString::number(home, 'g', 10)
	<<"\" min=\""<< QString::number(min, 'g', 10)<<"\" max=\""<< QString::number(max, 'g', 10)
	<< "\" tx=\""<< QString::number(backtX, 'g', 10) <<"\" ty=\""<< QString::number(backtY, 'g', 10) <<"\" tz=\""<< QString::number(backtZ, 'g', 10) 
	<<"\"  rx=\""<< QString::number(backrX, 'g', 10) <<"\" ry=\""<< QString::number(backrY, 'g', 10) <<"\" rz=\""<< QString::number(backrZ, 'g', 10) <<"\">\n";
	for (c=children.begin(); c!=children.end(); c++)
			(*c)->save(out, tabs+1);
	
	for (int i=0; i<tabs; i++) out << "\t";
	out << "</joint>\n";
	

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



float InnerModelJoint::setAngle(float angle, bool force)
{
	float ret;
	if ((angle <= max and angle >= min) or force)
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
		QString error;
		error.sprintf("internal error, no such axis %s\n", axis.c_str());
		throw error;
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



InnerModelNode * InnerModelJoint::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelJoint *ret;
	if (axis == "x")
	{
		ret = new InnerModelJoint(id, backlX, backlY, backlZ, backhX, backhY, backhZ, backtX, backtY, backtZ, backrZ, 0, 0, min, max, port, axis, home, (InnerModelTransform *)parent);
	}
	else if (axis == "y")
	{
		ret = new InnerModelJoint(id, backlX, backlY, backlZ, backhX, backhY, backhZ, backtX, backtY, backtZ, 0, backrZ, 0, min, max, port, axis, home, (InnerModelTransform *)parent);
	}
	else if (axis == "z")
	{
		ret = new InnerModelJoint(id, backlX, backlY, backlZ, backhX, backhY, backhZ, backtX, backtY, backtZ, 0, 0, backrZ, min, max, port, axis, home, (InnerModelTransform *)parent);
	}
	else
	{
		fprintf(stderr, "InnerModel internal error: invalid axis %s.\n", axis.c_str());
		exit(-1);
	}
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}


// ------------------------------------------------------------------------------------------------
// InnerModelPrismaticJoint
// ------------------------------------------------------------------------------------------------

InnerModelPrismaticJoint::InnerModelPrismaticJoint(QString id_, float min_, float max_, float val_, float offset_, uint32_t port_, std::string axis_, float home_, InnerModelTransform *parent_) : InnerModelTransform(id_,QString("static"),0,0,0,0,0,0, 0, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
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
		QString error;
		error.sprintf("internal error, no such axis %s\n", axis.c_str());
		throw error;
	}
	return ret;
}


InnerModelNode * InnerModelPrismaticJoint::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelPrismaticJoint *ret = new InnerModelPrismaticJoint(id, min, max, value, offset, port, axis, home, (InnerModelTransform *) parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}



// ------------------------------------------------------------------------------------------------
// InnerModelDifferentialRobot
// ------------------------------------------------------------------------------------------------

InnerModelDifferentialRobot::InnerModelDifferentialRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_, float noise_, bool collide_, InnerModelTransform *parent_) : InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	port = port_;
	noise = noise_;
	collide = collide_;
}

InnerModelNode * InnerModelDifferentialRobot::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelDifferentialRobot *ret = new InnerModelDifferentialRobot(id, backtX, backtY, backtZ, backrX, backrY, backrZ, port, noise, (InnerModelTransform *)parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}



// ------------------------------------------------------------------------------------------------
// InnerModelOmniRobot
// ------------------------------------------------------------------------------------------------

InnerModelOmniRobot::InnerModelOmniRobot(QString id_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, uint32_t port_, float noise_, bool collide_, InnerModelTransform *parent_) : InnerModelTransform(id_,QString("static"),tx_,ty_,tz_,rx_,ry_,rz_, 0, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	port = port_;
	noise = noise_;
	collide = collide_;
}

InnerModelNode * InnerModelOmniRobot::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelOmniRobot *ret = new InnerModelOmniRobot(id, backtX, backtY, backtZ, backrX, backrY, backrZ, port, noise, (InnerModelTransform *)parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}



// ------------------------------------------------------------------------------------------------
// InnerModelPlane
// ------------------------------------------------------------------------------------------------

InnerModelPlane::InnerModelPlane(QString id_, QString texture_, float width_, float height_,float depth_, int repeat_, float nx_, float ny_, float nz_, float px_, float py_, float pz_, bool collidable_, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	if ( abs(nx_)<0.001 and abs(ny_)<0.001 and abs(nz_)<0.001 ) nz_ = -1;
	normal = QVec::vec3(nx_, ny_, nz_);
	point = QVec::vec3(px_, py_, pz_);
	nx = ny = nz = px = py = pz = NULL;
	texture = texture_;
	width = width_;
	height = height_;
	depth = depth_;
	repeat = repeat_;
	collidable = collidable_;

#if FCL_SUPPORT==1
	std::vector<fcl::Vec3f> vertices;
	vertices.push_back(fcl::Vec3f(-width/2., +height/2., -depth/2.)); // Front NW
	vertices.push_back(fcl::Vec3f(+width/2., +height/2., -depth/2.)); // Front NE
	vertices.push_back(fcl::Vec3f(-width/2., -height/2., -depth/2.)); // Front SW
	vertices.push_back(fcl::Vec3f(+width/2., -height/2., -depth/2.)); // Front SE
	vertices.push_back(fcl::Vec3f(-width/2., +height/2., +depth/2.)); // Back NW
	vertices.push_back(fcl::Vec3f(+width/2., +height/2., +depth/2.)); // Back NE
	vertices.push_back(fcl::Vec3f(-width/2., -height/2., +depth/2.)); // Back SW
	vertices.push_back(fcl::Vec3f(+width/2., -height/2., +depth/2.)); // Back SE

	osg::Matrix r;
	r.makeRotate(osg::Vec3(0, 0, 1), osg::Vec3(normal(0), normal(1), -normal(2)));
	QMat qmatmat(4,4);
	for (int rro=0; rro<4; rro++)
	{
		for (int cco=0; cco<4; cco++)
		{
			qmatmat(rro,cco) = r(rro,cco);
		}
	}

	for (size_t i=0; i<vertices.size(); i++)
	{
		fcl::Vec3f v = vertices[i];
		const QVec rotated = (qmatmat*(QVec::vec3(v[0], v[1], v[2]).toHomogeneousCoordinates())).fromHomogeneousCoordinates();
		vertices[i] = fcl::Vec3f(rotated(0)+px_, rotated(1)+py_, rotated(2)+pz_);
	}

	std::vector<fcl::Triangle> triangles;
	triangles.push_back(fcl::Triangle(0,1,2)); // Front
	triangles.push_back(fcl::Triangle(1,2,3));
	triangles.push_back(fcl::Triangle(4,5,6)); // Back
	triangles.push_back(fcl::Triangle(5,6,7));
	triangles.push_back(fcl::Triangle(4,0,6)); // Left
	triangles.push_back(fcl::Triangle(0,6,2));
	triangles.push_back(fcl::Triangle(5,1,7)); // Right
	triangles.push_back(fcl::Triangle(1,7,3));
	triangles.push_back(fcl::Triangle(5,1,4)); // Top
	triangles.push_back(fcl::Triangle(1,4,0));
	triangles.push_back(fcl::Triangle(2,3,6)); // Bottom
	triangles.push_back(fcl::Triangle(3,6,7));

////
////   UNCOMMENT THIS CODE TO GENERATE A POINTCLOUD OF THE POINTS IN THE MESHES
////
// std::ofstream outputFile;
// outputFile.open((id.toStdString()+".pcd").c_str());
// outputFile << "# .PCD v.7 - Point Cloud Data file format\n";
// outputFile << "VERSION .7\n";
// outputFile << "FIELDS x y z \n";
// outputFile << "SIZE 4 4 4\n";
// outputFile << "TYPE F F F\n";
// outputFile << "COUNT 1 1 1\n";
// outputFile << "WIDTH " << vertices.size() << "\n";
// outputFile << "HEIGHT 1\n";
// outputFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
// outputFile << "POINTS " << vertices.size() << "\n";
// outputFile << "DATA ascii\n";
// for (size_t i=0; i<vertices.size(); i++)
// {
// 	outputFile << vertices[i][0]/1000. << " " << vertices[i][1]/1000. << " " << vertices[i][2]/1000. << "\n";
// }
// outputFile.close();


	fclMesh = FCLModelPtr(new FCLModel());
	fclMesh->beginModel();
	fclMesh->addSubModel(vertices, triangles);
	fclMesh->endModel();
	collisionObject = new fcl::CollisionObject(fclMesh);
	
#endif
}



void InnerModelPlane::print(bool verbose)
{
	if (verbose) normal.print(QString("Plane: ")+id);
}



void InnerModelPlane::save(QTextStream &out, int tabs)
{
// 	float width, height, depth;
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<plane id=\"" << id << "\" texture=\"" << texture << "\" size=\"" << QString::number(width,'g', 10)<<","<<QString::number( height,'g', 10)<<","
	<<QString::number( depth,'g', 10) << "\" repeat=\"" << QString::number(repeat, 'g', 10) << "\" nx=\"" << QString::number(normal(0), 'g', 10) 
	<< "\" ny=\"" << QString::number(normal(1), 'g', 10) << "\" nz=\"" 
	<< QString::number(normal(2), 'g', 10) << "\" px=\"" << QString::number(point(0), 'g', 10) << "\" py=\"" << QString::number(point(1), 'g', 10) 
	<< "\" pz=\"" << QString::number(point(2), 'g', 10) <<"\" collide=\""<< QString::number(collidable,'g',10)<< "\" />\n";
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

InnerModelNode * InnerModelPlane::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelPlane *ret = new InnerModelPlane(id, texture, width, height, depth, repeat, normal(0), normal(1), normal(2), point(0), point(1), point(2), parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}



// ------------------------------------------------------------------------------------------------
// InnerModelCamera
// ------------------------------------------------------------------------------------------------

InnerModelCamera::InnerModelCamera(QString id_, float width_, float height_, float focal_, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	camera = Cam(focal_, focal_, width_/2., height_/2.);
	camera.setSize(width, height);
// 	camera.print(id_);
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
	out << "<camera id=\"" << id << "\" width=\"" << QString::number(width, 'g', 10) << "\" height=\"" << QString::number(height, 'g', 10) << "\" focal=\"" << QString::number(camera.getFocal(), 'g', 10) << "\" />\n";
}



void InnerModelCamera::update()
{
	if (fixed)
	{
	}
	updateChildren();
}

InnerModelNode * InnerModelCamera::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelCamera *ret = new InnerModelCamera(id, width, height, focal, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	ret->camera = camera;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}



// ------------------------------------------------------------------------------------------------
// InnerModelRGBD
// ------------------------------------------------------------------------------------------------

InnerModelRGBD::InnerModelRGBD(QString id_, float width, float height, float focal, float _noise, uint32_t _port, QString _ifconfig, InnerModelNode *parent_) : InnerModelCamera(id_, width, height, focal, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	noise = _noise;
	port = _port;
	ifconfig = _ifconfig;
}



void InnerModelRGBD::save(QTextStream &out, int tabs)
{
	
// 	<rgbd id="laser" focal="120" width="160" height="120" port="10097" ifconfig="10068,10004" />
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<rgbd id=\"" << id << "\" width=\"" <<QString::number( width, 'g', 10) << "\" height=\"" <<QString::number(  height, 'g', 10)  << "\" focal=\"" << QString::number(camera.getFocal(), 'g', 10)
	<<"\" port=\""<<port<<"\" ifconfig=\""<<ifconfig<<"\" noise=\""<<QString::number(noise, 'g', 10)<< "\" />\n";
}


InnerModelNode * InnerModelRGBD::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelRGBD *ret = new InnerModelRGBD(id, width, height, focal, noise, port, ifconfig, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}


// ------------------------------------------------------------------------------------------------
// InnerModelIMU
// ------------------------------------------------------------------------------------------------

InnerModelIMU::InnerModelIMU(QString id_, uint32_t _port, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
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


InnerModelNode * InnerModelIMU::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelIMU *ret = new InnerModelIMU(id, port, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}


// ------------------------------------------------------------------------------------------------
// InnerModelLaser
// ------------------------------------------------------------------------------------------------

InnerModelLaser::InnerModelLaser(QString id_, uint32_t _port, uint32_t _min, uint32_t _max, float _angle, uint32_t _measures, QString _ifconfig, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
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

	out << "<laser id=\"" << id <<"\" port=\""<<port<<"\" min=\""<< QString::number(min,'g',10)<<"\" max=\""<<QString::number(max,'g',10)
	<<"\" measures=\""<<QString::number(measures,'g',10)<<"\" angle=\""<<QString::number(angle,'g',10)
	<<"\" ifconfig=\""<<ifconfig<< "\" />\n";
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

InnerModelNode * InnerModelLaser::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelLaser *ret = new InnerModelLaser(id, port, min, max, angle, measures, ifconfig, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}


// ------------------------------------------------------------------------------------------------
// InnerModelMesh
// ------------------------------------------------------------------------------------------------

InnerModelMesh::InnerModelMesh(QString id_, QString meshPath_, float scale, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable,  InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	InnerModelMesh(id_,meshPath_,scale,scale,scale,render_,tx_,ty_,tz_,rx_,ry_,rz_, collidable, parent_);
}



InnerModelMesh::InnerModelMesh(QString id_, QString meshPath_, float scalex_, float scaley_, float scalez_, RenderingModes render_, float tx_, float ty_, float tz_, float rx_, float ry_, float rz_, bool collidable_, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
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
	collidable = collidable_;

#if FCL_SUPPORT==1
	// Get to the OSG geode
	//osg::Node *osgnode_ = osgDB::readNodeFile(meshPath.toStdString());
	osg::ref_ptr<osg::Node> osgnode_ = osgDB::readNodeFile(meshPath.toStdString()); 
	if (not osgnode_) printf("Could not open: '%s'.\n", meshPath.toStdString().c_str());
	if (osgnode_ != NULL)
	{
		// Instanciate the vector of vertices and triangles (that's what we are looking for)
		std::vector<fcl::Vec3f> vertices;
		std::vector<fcl::Triangle> triangles;
		CalculateTriangles calcTriangles(&vertices, &triangles);
		osgnode_->accept(calcTriangles);

// 		printf("id: %s\n", id.toStdString().c_str());
// 		printf("scale: %f %f %f\n", scalex, scaley, scalez);
// 		printf("points: %zu\n", vertices.size());
// 		printf("triangles: %zu\n", triangles.size());

		// Get the internal transformation matrix of the mesh
		RTMat rtm(rx, ry, rz, tx, ty, tz);
		// Transform each of the read vertices
		for (size_t i=0; i<vertices.size(); i++)
		{
			fcl::Vec3f v = vertices[i];
			const QMat v2 = (rtm * QVec::vec3(v[0]*scalex, v[1]*scaley, -v[2]*scalez).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
			vertices[i] = fcl::Vec3f(v2(0), v2(1), v2(2));
		}

// ////
// ////   UNCOMMENT THIS CODE TO GENERATE A POINTCLOUD OF THE POINTS IN THE MESHES
// ////
// std::ofstream outputFile;
// outputFile.open((id.toStdString()+".pcd").c_str());
// outputFile << "# .PCD v.7 - Point Cloud Data file format\n";
// outputFile << "VERSION .7\n";
// outputFile << "FIELDS x y z \n";
// outputFile << "SIZE 4 4 4\n";
// outputFile << "TYPE F F F\n";
// outputFile << "COUNT 1 1 1\n";
// outputFile << "WIDTH " << vertices.size() << "\n";
// outputFile << "HEIGHT 1\n";
// outputFile << "VIEWPOINT 0 0 0 1 0 0 0\n";
// outputFile << "POINTS " << vertices.size() << "\n";
// outputFile << "DATA ascii\n";
// for (size_t i=0; i<vertices.size(); i++)
// {
// 	outputFile << vertices[i][0]/1000. << " " << vertices[i][1]/1000. << " " << vertices[i][2]/1000. << "\n";
// }
// outputFile.close();


		// Associate the read vertices and triangles vectors to the FCL collision model object
		fclMesh = FCLModelPtr(new FCLModel());
		fclMesh->beginModel();
		fclMesh->addSubModel(vertices, triangles);
		fclMesh->endModel();
		collisionObject = new fcl::CollisionObject(fclMesh);
		
	}
	else
	{
		QString error;
		error.sprintf("Failed to read mesh \"%s\" for collision support!\n", meshPath.toStdString().c_str());
		throw error;
	}
#endif
}


void InnerModelMesh::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<mesh id=\""<<id<<"\"" <<" file=\"" << meshPath 
	<< "\" scale=\"" << QString::number(scalex, 'g', 10) << ","<< QString::number(scaley, 'g', 10)<< ","<< QString::number(scalez, 'g', 10) 
	<< "\" tx=\"" << QString::number(tx, 'g', 10) << "\" ty=\"" << QString::number(ty, 'g', 10) << "\" tz=\"" << QString::number(tz, 'g', 10) 
	<< "\" rx=\"" << QString::number(rx, 'g', 10) << "\" ry=\"" << QString::number(ry, 'g', 10) << "\" rz=\"" << QString::number(rz, 'g', 10) 
	<<"\" collide=\""<< QString::number(collidable,'g',10)<< "\" />\n";
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



InnerModelNode * InnerModelMesh::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelMesh *ret = new InnerModelMesh(id, meshPath, scalex, scaley, scalez, render, tx, ty, tz, rx, ry, rz, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

#if FCL_SUPPORT==1
	// Associate the read vertices and triangles vectors to the FCL collision model object
	ret->fclMesh = FCLModelPtr(new FCLModel(*fclMesh.get()));
	ret->collisionObject = new fcl::CollisionObject(ret->fclMesh);
#endif

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}


// ------------------------------------------------------------------------------------------------
// InnerModelPointCloud
// ------------------------------------------------------------------------------------------------

InnerModelPointCloud::InnerModelPointCloud(QString id_, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
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



InnerModelNode * InnerModelPointCloud::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelPointCloud *ret = new InnerModelPointCloud(id, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}



// ------------------------------------------------------------------------------------------------
// InnerModelTouchSensor
// ------------------------------------------------------------------------------------------------

InnerModelTouchSensor::InnerModelTouchSensor(QString id_, QString stype_, float nx_, float ny_, float nz_, float min_, float max_, uint32_t port_, InnerModelNode *parent_) : InnerModelNode(id_, parent_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif

	id = id_;
	nx = nx_;
	ny = ny_;
	nz = nz_;
	min = min_;
	max = max_;
	stype = stype_;
	port = port_;
}

InnerModelNode * InnerModelTouchSensor::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelTouchSensor *ret = new InnerModelTouchSensor(id, stype, nx, ny, nz, min, max, port, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
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
		printf("No node %s\n", a.toStdString().c_str());
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
// 	fcl::AABB a1 = n1->collisionObject->getAABB();
// 	fcl::Vec3f v1 = a1.center();

	n2->collisionObject->computeAABB();
// 	fcl::AABB a2 = n2->collisionObject->getAABB();
// 	fcl::Vec3f v2 = a2.center();

// 	qDebug()<< a;
// 	printf("- (%f,  %f,  %f) --- (%f,  %f,  %f) [%f , %f , %f]  <<%f %d>>\n", v1[0], v1[1], v1[2], (v1-v2)[0], (v1-v2)[1], (v1-v2)[2], a1.width(), a1.height(), a1.depth(), a1.distance(a2), a1.overlap(a2));
// 	qDebug()<< b;
// 	printf("- (%f,  %f,  %f) --- (%f,  %f,  %f) [%f , %f , %f]  <<%f %d>>\n", v2[0], v2[1], v2[2], (v1-v2)[0], (v1-v2)[1], (v1-v2)[2], a2.width(), a2.height(), a2.depth(), a1.distance(a2), a1.overlap(a2));

	// NOTE: Un poco de documentacion nunca esta mal, sabeis --> http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/namespacefcl.html
	// std::size_t 	collide (const CollisionObject *o1, const CollisionObject *o2, const CollisionRequest &request, CollisionResult &result)
	fcl::collide(                  n1->collisionObject,       n2->collisionObject,                         request,                  result);
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
