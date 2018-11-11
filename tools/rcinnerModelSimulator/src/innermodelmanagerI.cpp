/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#include "innermodelmanagerI.h"
#include "specificworker.h"


InnerModelManagerI::InnerModelManagerI(std::shared_ptr<SpecificWorker> w)
{
	worker = w;
}


bool InnerModelManagerI::setPose(const std::string &base, const std::string &item, const RoboCompInnerModelManager::Pose3D &pose, const Ice::Current&)
{
	guard gl(worker->innerModel->mutex);
	
	QString qBase = QString::fromStdString(base);
	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::setPose()";

	//check type transform
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(worker->getNode(qBase, m));
	worker->checkOperationInvalidNode(aux, m + qBase +"can't be use as base because it's not a InnerModelTransform node.");
	aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(worker->getNode(qItem, m));
	worker->checkOperationInvalidNode(aux, m + qItem +"can't be use as item because it's not a InnerModelTransform node.");

	worker->innerModel->updateTransformValues(qItem, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, qBase);

	return true;
}


bool InnerModelManagerI::setPoseFromParent(const std::string &item, const RoboCompInnerModelManager::Pose3D &pose, const Ice::Current&)
{
	//return worker->imm_setPoseFromParent(id, item, pose);
	guard gl(worker->innerModel->mutex);

	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::setPose()";

	//check type transform
	InnerModelTransform *aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(worker->getNode(qItem, m));
	worker->checkOperationInvalidNode(aux, m + qItem +"can't be use as item because it's not a InnerModelTransform node.");
	worker->innerModel->updateTransformValues(qItem, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);

	return true;
}

bool InnerModelManagerI::getPose(const std::string &base, const std::string &item, RoboCompInnerModelManager::Pose3D &pose, const Ice::Current&)
{
	//return worker->imm_getPose(id, base, item, pose);
	guard gl(worker->innerModel->mutex);

	QVec p;
	QString qBase = QString::fromStdString(base);
	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::getPose()";

	// check type transform
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(worker->getNode(qBase, m));
	worker->checkOperationInvalidNode(aux, m + qBase +"can't be use as base because it's not a InnerModelTransform node.");
	aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(worker->getNode(qItem, m));
	worker->checkOperationInvalidNode(aux, m + qItem +"can't be use as item because it's not a InnerModelTransform node.");

	// calculate position
	p = worker->innerModel->transform(QString::fromUtf8(base.c_str()), QVec::vec3(0,0,0), QString::fromUtf8(item.c_str()));
	pose.x = p.x();
	pose.y = p.y();
	pose.z = p.z();
	//calulate rotation
	p = worker->innerModel->getRotationMatrixTo(QString::fromUtf8(base.c_str()), QString::fromUtf8(item.c_str())).extractAnglesR();
	pose.rx = p.x();
	pose.ry = p.y();
	pose.rz = p.z();

	return true;
}


bool InnerModelManagerI::getPoseFromParent(const std::string &item, RoboCompInnerModelManager::Pose3D &pose, const Ice::Current&)
{
	//return worker->imm_getPoseFromParent(id, item, pose);
	guard gl(worker->innerModel->mutex);

	QString m="RoboCompInnerModelManager::getPoseFromParent()";

	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(worker->getNode(QString::fromStdString(item), m));
	worker->checkOperationInvalidNode(aux, m+aux->id+"can't be use as base because it's not a InnerModelTransform node.");

	pose.x = aux->backtX;
	pose.y = aux->backtY;
	pose.z = aux->backtZ;
	pose.rx = aux->backrX;
	pose.ry = aux->backrY;
	pose.rz = aux->backrZ;

	return true;
}


bool InnerModelManagerI::transform(const std::string &base, const std::string &item, const coord3D &coordInItem, coord3D &coordInBase, const Ice::Current&)
{
	//return worker->imm_transform(id, base, item, coordInItem, coordInBase);
	guard gl(worker->innerModel->mutex);

	QVec p;
	const QString qBase = QString::fromStdString(base);
	const QString qItem = QString::fromStdString(item);
	const QString m="RoboCompInnerModelManager::transform()";

	//check type transform
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(worker->getNode(qBase, m));
	worker->checkOperationInvalidNode(aux, m + qBase +"can't be used as base because it's not a InnerModelTransform node.");

	aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(worker->getNode(qItem, m));
	worker->checkOperationInvalidNode(aux, m + qItem +"can't be used as item because it's not a InnerModelTransform node.");

	// calculate position
	p = worker->innerModel->transform(QString::fromUtf8(base.c_str()), QVec::vec3(coordInItem.x,coordInItem.y,coordInItem.z),QString::fromUtf8(item.c_str()));
	coordInBase.x = p.x();
	coordInBase.y = p.y();
	coordInBase.z = p.z();

	return true;
}

Matrix InnerModelManagerI::getTransformationMatrix(const std::string &base, const std::string &item, const Ice::Current&)
{
	//return worker->imm_getTransformationMatrix(base, item);
	const QString qBase = QString::fromStdString(base);
	const QString qItem = QString::fromStdString(item);
	const QString m="RoboCompInnerModelManager::transform()";
	guard gl(worker->innerModel->mutex);


	//check type transform
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(worker->getNode(qBase, m));
	worker->checkOperationInvalidNode(aux, m + qBase +"can't be used as base because it's not a InnerModelTransform node.");

	aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(worker->getNode(qItem, m));
	worker->checkOperationInvalidNode(aux, m + qItem +"can't be used as item because it's not a InnerModelTransform node.");

	// calculate position
	RTMat retA = worker->innerModel->getTransformationMatrix(qBase, qItem);
	RoboCompInnerModelManager::Matrix retB;

	retB.cols = retA.nCols();
	retB.rows = retA.nRows();
	retB.data.resize(retB.cols*retB.rows);
	retA.print("retA");
	printf("\n");
	for (int r=0; r<retB.rows; r++)
	{
		for (int c=0; c<retB.cols; c++)
		{
			printf("%f ", retB.data[r*retB.cols + c]);
			retB.data[r*retB.cols + c] = retA(r, c);
		}
		printf("\n");
	}
	printf("\n");

	return retB;
}


bool InnerModelManagerI::setScale(const std::string &item, float scaleX,float scaleY, float scaleZ, const Ice::Current&)
{
	//return worker->imm_setScale(id, item, scaleX, scaleY, scaleZ);
	guard gl(worker->innerModel->mutex);

	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::setScale()";

	InnerModelMesh *aux = dynamic_cast<InnerModelMesh*>(worker->getNode(QString::fromStdString(item),m));
	worker->checkOperationInvalidNode(aux,m + qItem +"can't be used because it's not a InnerModelMesh node.");

	aux->setScale(scaleX, scaleY, scaleZ);
	return true;
}


bool InnerModelManagerI::setPlane(const std::string &item, const RoboCompInnerModelManager::Plane3D &plane, const Ice::Current&)
{
	//return worker->imm_setPlane(id, item, plane);
	guard gl(worker->innerModel->mutex);

	QString m="RoboCompInnerModelManager::setPlane()";
// 	printf("SETPLANE %s: %f_%f_%f\n", item.c_str(), plane.px, plane.py, plane.pz);
	InnerModelPlane *aux = dynamic_cast<InnerModelPlane*>(worker->getNode(QString::fromStdString(item), m));
	worker->checkOperationInvalidNode(aux,m + aux->id +"can't be use as base because it's not of the type InnerModelPlane.");
	worker->innerModel->updatePlaneValues(QString::fromStdString(item), plane.nx, plane.ny, plane.nz, plane.px, plane.py, plane.pz);
	return true;
	
}
bool InnerModelManagerI::setPlaneTexture(const std::string &item, const std::string &texture, const Ice::Current&)
{
	//return worker->imm_setPlaneTexture(id, item, texture);
	guard gl(worker->innerModel->mutex);

	QString m="RoboCompInnerModelManager::setPlaneTextureº()";
	printf("SETPLANETEXTURE %s: %s\n", item.c_str(), texture.c_str());
	InnerModelPlane *aux = dynamic_cast<InnerModelPlane*>(worker->getNode(QString::fromStdString(item), m));
	//qDebug()<<"aux->texture"<<aux->texture<<"qstring"<<QString::fromStdString(texture);
	
	aux->texture=QString::fromStdString(texture);
	
	osg::Image *image=NULL;
	image = osgDB::readImageFile(texture);
	if (not image)
	{
		qDebug() << "Couldn't load texture:" << texture.c_str();
		throw "Couldn't load texture.";
	}
	
	worker->imv->planesHash[aux->id]->image =image;
	worker->imv->planesHash[aux->id]->texture->setImage(image);

	qDebug()<<"change aux->texture"<<aux->texture;
// 	checkOperationInvalidNode(aux,m + aux->id +"can't be use as base because it's not of the type InnerModelPlane.");
// 	innerModel->updatePlaneValues(QString::fromStdString(item), plane.nx, plane.ny, plane.nz, plane.px, plane.py, plane.pz);
	return true;
}

bool InnerModelManagerI::addTransform(const std::string &item, const std::string &engine, const std::string &base,const Pose3D &pose, const Ice::Current&)
{
	//return worker->imm_addTransform(id, item, engine, base, pose);
	guard gl(worker->innerModel->mutex);

	InnerModelNode *parent = worker->getNode(QString::fromStdString(base), "RoboCompInnerModelManager::addTransform()");
	worker->checkNodeAlreadyExists(QString::fromStdString(item), "RoboCompInnerModelManager::addTransform()");
	
	QString qEngine = QString::fromStdString( engine);
	if (qEngine !="static" and qEngine !="bullet")
	{
		qEngine = "static";
	}

	InnerModelTransform *tr = worker->innerModel->newTransform(QString::fromStdString(item), QString::fromStdString("static") ,parent, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
	parent->addChild(tr);
	worker->imv->recursiveConstructor(tr, worker->imv->mts[parent->id], worker->imv->mts, worker->imv->meshHash); // imv->osgmeshes,imv->osgmeshPats);
	
#ifdef INNERMODELMANAGERDEBUG
	qDebug()<<"engine"<<qEngine;
	qDebug() <<"transform: pose.x<<pose.y<<pose.z"<<pose.x<<pose.y<<pose.z<<QString::fromStdString(item);
#endif
	return true;
}


bool InnerModelManagerI::addJoint(const std::string &item, const std::string &base, const jointType &j, const Ice::Current&)
{
	//return worker->imm_addJoint(id, item, base, j);
	RoboCompInnerModelManager::jointType j_ = j;
	if (j_.axis == "")
	{
		j_.axis = "z";
	}

	guard gl(worker->innerModel->mutex);

	RoboCompInnerModelManager::Pose3D pose = j_.pose;

	InnerModelTransform *parent=dynamic_cast<InnerModelTransform *>(worker->getNode(QString::fromStdString(base), "RoboCompInnerModelManager::addJoint()"));
	worker->checkNodeAlreadyExists(QString::fromStdString(item), "RoboCompInnerModelManager::addJoint()");

	InnerModelJoint *j_N = worker->innerModel->newJoint(QString::fromStdString(item), parent, j_.lx, j_.ly, j_.lz, j_.hx, j_.hy, j_.hz, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, j_.min, j_.max, j_.port, j_.axis);
	parent->addChild (j_N);

	// Create Interface in case the port is not 0
	if (j_N->port != 0)
	{ 
		//worker->addJM(j_N);  //CHANGE
		/*if (worker->servers.jm_servers.count(j_N->port) == 0)*/
		if (worker->servers.hMaps.count<InnerModelJoint>(j_N->port) == 0)
			worker->servers.hMaps.insert(j_N->port, JointMotorServer(worker->communicator, worker, j_N->port));
			//worker->servers.jm_servers.insert(std::pair<uint32_t, JointMotorServer>(j_N->port, JointMotorServer(worker->communicator, worker, j_N->port)));
		
		worker->servers.hMaps.at<JointMotorServer>(j_N->port).add(j_N);
		//worker->servers.jm_servers.at(j_N->port).add(j_N);
	}
	worker->imv->recursiveConstructor(j_N, worker->imv->mts[parent->id], worker->imv->mts, worker->imv->meshHash); // imv->osgmeshes,imv->osgmeshPats);

	return true;
}


bool InnerModelManagerI::addMesh(const std::string &item,const std::string &base,const meshType &m, const Ice::Current&)
{
	//return worker->imm_addMesh(id, item, base, m);
		guard gl(worker->innerModel->mutex);

	QString msg="RoboCompInnerModelManager::addMesh()";
#ifdef INNERMODELMANAGERDEBUG
	qDebug() <<msg<<QString::fromStdString(base) <<QString::fromStdString(item);
	qDebug() <<QString::fromStdString(m.meshPath);
#endif
	InnerModelTransform *parent = dynamic_cast<InnerModelTransform*>(worker->getNode(QString::fromStdString(base), msg));

	//Checking if its parent is not a mesh.
	worker->checkOperationInvalidNode(parent, msg);
	worker->checkNodeAlreadyExists(QString::fromStdString(item), msg);
	worker->checkInvalidMeshValues(m,msg);

	int render = m.render;
	if(render!=0 and render!=1)
	{
		render=0;
	}
	
	InnerModelMesh *mesh = worker->innerModel->newMesh (
		QString::fromStdString(item),
		parent,
		QString::fromStdString(m.meshPath),
		m.scaleX, m.scaleY, m.scaleZ,
		render,
		m.pose.x, m.pose.y, m.pose.z,
		m.pose.rx, m.pose.ry, m.pose.rz);

	mesh->setScale(m.scaleX, m.scaleY, m.scaleZ);
	parent->addChild(mesh);

	worker->imv->recursiveConstructor(mesh, worker->imv->mts[parent->id], worker->imv->mts, worker->imv->meshHash); // osgmeshes,imv->osgmeshPats);
	return true;
}


bool InnerModelManagerI::addPlane(const std::string &item, const std::string &base, const Plane3D &p, const Ice::Current&)
{
	//return worker->imm_addPlane(id, item, base, p);
	guard gl(worker->innerModel->mutex);


	InnerModelNode *parent = worker->getNode(QString::fromStdString(base), "RoboCompInnerModelManager::addPlane()");
	worker->checkNodeAlreadyExists(QString::fromStdString(item), "RoboCompInnerModelManager::addPlane()");


	InnerModelPlane *plane = worker->innerModel->newPlane(QString::fromStdString(item), parent, QString::fromStdString(p.texture),
	                         p.width, p.height, p.thickness, 1,
	                         p.nx, p.ny, p.nz, p.px, p.py, p.pz);
	parent->addChild(plane);

	worker->imv->recursiveConstructor(plane, worker->imv->mts[parent->id], worker->imv->mts, worker->imv->meshHash);
	//I think not necessary	
	// 	imv->update();

	return true;
}


bool InnerModelManagerI::removeNode(const std::string &item, const Ice::Current&)
{
	guard gl(worker->innerModel->mutex);

	QString msg="RoboCompInnerModelManager::removeNode()";
#ifdef INNERMODELMANAGERDEBUG
	qDebug() << msg << QString::fromStdString(item);
#endif

	QString id =QString::fromStdString(item);
	if(id == "world" || id == "root") 
	{
#ifdef INNERMODELMANAGERDEBUG
		qDebug() << msg << id <<"Can't remove root elements";
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::OperationInvalidNode;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error, cannot remove. Node: " <<id.toStdString();
		err.text = oss.str();
		throw err;
	}

	InnerModelNode *node = worker->getNode(QString::fromStdString(item), msg);
	worker->checkOperationInvalidNode(node,msg);

	QStringList l;
	l.clear();
	
	worker->innerModel->getSubTree(node,&l);
// 	qDebug()<<"----------- l.size()"<<l.size();
	///remove handlers and node
	foreach (QString n, l)
	{
		///remove handlers
#ifdef INNERMODELMANAGERDEBUG
 		qDebug()<<"remove"<<n;
#endif
		InnerModelJoint *jN = dynamic_cast<InnerModelJoint *> (worker->innerModel->getNode(n));
		if (jN!=NULL && jN->port!=0)
		{
#ifdef INNERMODELMANAGERDEBUG		
 			qDebug()<<"remove Joint"<<n<<jN->port;
#endif
			worker->servers.removeJointMotorServer(jN);
		}
		
		///remove nodes in InnerModel tree
// 		innerModel->removeNode(n);
	}
	worker->innerModel->removeSubTree(node,&l);
	

	/// Replicate InnerModel node removals in the InnerModelViewer tree. And in handlers Lists
	foreach(QString n, l) {
		/// Replicate plane removals
		if(worker->imv->meshHash.contains(n)) {
// 			qDebug()<<"/// Replicate meshHash removals"<<n;			
			while(worker->imv->meshHash[n].osgmeshPaths->getNumParents() > 0)
				( worker->imv->meshHash[n].osgmeshPaths->getParent(0))->removeChild(worker->imv->meshHash[n].osgmeshPaths);			
			while(worker->imv->meshHash[n].osgmeshes->getNumParents() > 0)
				( worker->imv->meshHash[n].osgmeshes->getParent(0))->removeChild(worker->imv->meshHash[n].osgmeshes);
			while(worker->imv->meshHash[n].meshMts->getNumParents() > 0)	
				( worker->imv->meshHash[n].meshMts->getParent(0))->removeChild(worker->imv->meshHash[n].meshMts);			
				
			worker->imv->meshHash.remove(n);
// 			meshColision.remove(n);
		}
		/// Replicate transform removals
		if(worker->imv->mts.contains(n)) {
//			qDebug()<<"/// Replicate transform removals";//<<n<<imv->mts[n]->getNumParents();
 			while(worker->imv->mts[n]->getNumParents() > 0) {
				(worker->imv->mts[n]->getParent(0))->removeChild(worker->imv->mts[n]);
 			}			
 			worker->imv->mts.remove(n);
		}
		/// Replicate plane removals
		if(worker->imv->planeMts.contains(n)) {
//			qDebug()<<"/// Replicate plane removals";
			while(worker->imv->planeMts[n]->getNumParents() > 0) {
				((osg::Group *)(worker->imv->planeMts[n]->getParent(0)))->removeChild(worker->imv->planeMts[n]);
			}
			worker->imv->planeMts.remove(n);
			worker->imv->planesHash.remove(n);
		}
	}
	return true;
}

bool InnerModelManagerI::moveNode(const std::string &src, const std::string &dst, const Ice::Current&)
{
//	return worker->imm_moveNode(id, src,dst);
		guard gl(worker->innerModel->mutex);

	QString msg="RoboCompInnerModelManager::moveNode()";
#ifdef INNERMODELMANAGERDEBUG
	qDebug() <<msg<<QString::fromStdString(src)<<QString::fromStdString(dst);
#endif

	QString idSrc =QString::fromStdString(src);
	QString idDst =QString::fromStdString(dst);
	if(idSrc=="world" || idSrc=="root" ) {
#ifdef INNERMODELMANAGERDEBUG
		qDebug() <<msg<<idSrc<<"Can't move root elements";
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::OperationInvalidNode;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error, cannot move Node: " <<idSrc.toStdString();
		err.text = oss.str();
		throw err;
	}
	
	if(idDst =="root" ) {
#ifdef INNERMODELMANAGERDEBUG
		qDebug() <<msg<<idDst<<"Can't move to root elements";
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::OperationInvalidNode;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" Forbidden, cannot move Node: " <<idSrc.toStdString()<<"to"<<idDst.toStdString()<<"element";
		err.text = oss.str();
		throw err;
	}
	

	InnerModelNode *nodeSrc = worker->getNode(idSrc, msg);
	worker->checkOperationInvalidNode(nodeSrc,msg);
	
	InnerModelNode *nodeDst = worker->getNode(idDst, msg);
	worker->checkOperationInvalidNode(nodeDst,msg);

	QStringList l;
	l.clear();
	
	//consigo ids para viewer
	worker->innerModel->getSubTree (nodeSrc,&l);
	//muevo 
	worker->innerModel->moveSubTree(nodeSrc,nodeDst);
	
	
	

	/// Replicate InnerModel node removals in the InnerModelViewer tree. And in handlers Lists
	foreach(QString n, l) {
		/// Replicate plane removals
		if(worker->imv->meshHash.contains(n)) {
// 			qDebug()<<"/// Replicate meshHash removals"<<n;			
			while(worker->imv->meshHash[n].osgmeshPaths->getNumParents() > 0)
				( worker->imv->meshHash[n].osgmeshPaths->getParent(0))->removeChild(worker->imv->meshHash[n].osgmeshPaths);			
			while(worker->imv->meshHash[n].osgmeshes->getNumParents() > 0)
				( worker->imv->meshHash[n].osgmeshes->getParent(0))->removeChild(worker->imv->meshHash[n].osgmeshes);
			while(worker->imv->meshHash[n].meshMts->getNumParents() > 0)	
				( worker->imv->meshHash[n].meshMts->getParent(0))->removeChild(worker->imv->meshHash[n].meshMts);			
				
			worker->imv->meshHash.remove(n);
// 			meshColision.remove(n);
		}
		/// Replicate transform removals
		if(worker->imv->mts.contains(n)) {
//			qDebug()<<"/// Replicate transform removals";//<<n<<imv->mts[n]->getNumParents();
 			while(worker->imv->mts[n]->getNumParents() > 0) {
				(worker->imv->mts[n]->getParent(0))->removeChild(worker->imv->mts[n]);
 			}			
 			worker->imv->mts.remove(n);
		}
		/// Replicate plane removals
		if(worker->imv->planeMts.contains(n)) {
//			qDebug()<<"/// Replicate plane removals";
			while(worker->imv->planeMts[n]->getNumParents() > 0) {
				((osg::Group *)(worker->imv->planeMts[n]->getParent(0)))->removeChild(worker->imv->planeMts[n]);
			}
			worker->imv->planeMts.remove(n);
			worker->imv->planesHash.remove(n);
		}
		
	}
	foreach(QString n, l) 
	{
		
		worker->imv->recursiveConstructor(worker->innerModel->getNode(n), worker->imv->mts[worker->innerModel->getNode(n)->parent->id], worker->imv->mts, 
																			worker->imv->meshHash); // imv->osgmeshes,imv->osgmeshPats);
	}
	
	qDebug()<<"-- fin move Subtree --";
	return true;
}


bool InnerModelManagerI::addAttribute(const string &idNode, const string &name, const string &type, const string &value, const Ice::Current&)
{
///	return worker->imm_addAttribute(id, idNode, name, type, value);
	guard gl(worker->innerModel->mutex);

	QString qIdNode=QString::fromStdString(idNode);
	QString qName=QString::fromStdString(name);
	QString qType=QString::fromStdString(type);
	QString qValue=QString::fromStdString(value);
	QString m="RoboCompInnerModelManager::addAttribute()";

	InnerModelNode *node = worker->getNode(qIdNode, m);
	worker->AttributeAlreadyExists(node,qName,m);

	InnerModelNode::AttributeType t;
	t.type=qType;
	t.value=qValue;
	node->attributes.insert(qName,t);

	return true;
}


bool InnerModelManagerI::removeAttribute(const string &idNode, const string &name, const Ice::Current&)
{
	//return worker->imm_removeAttribute(id, idNode, name);
	guard gl(worker->innerModel->mutex);

	QString qIdNode=QString::fromStdString(idNode);
	QString qName=QString::fromStdString(name);
	QString m="RoboCompInnerModelManager::removeAttribute()";

	InnerModelNode *node = worker->getNode(qIdNode, m);
	worker->NonExistingAttribute(node, qName,m);

	node->attributes.remove(qName);

	return true;
}


bool InnerModelManagerI::setAttribute(const string &idNode, const string &name, const string &type, const string &value, const Ice::Current&)
{
	//return worker->imm_setAttribute(id, idNode, name, type, value);
	guard gl(worker->innerModel->mutex);

	QString qIdNode=QString::fromStdString(idNode);
	QString qName=QString::fromStdString(name);
	QString qType=QString::fromStdString(type);
	QString qValue=QString::fromStdString(value);

	QString m="RoboCompInnerModelManager::setAttribute()";

	InnerModelNode *node = worker->getNode(qIdNode, m);
	worker->NonExistingAttribute(node,qName,m);

	node->attributes[qName].type=qType;
	node->attributes[qName].value=qValue;

	return true;
}


bool InnerModelManagerI::getAttribute(const string &idNode, const string &name,  string &type,  string &value, const Ice::Current&)
{
	//return worker->imm_getAttribute(id, idNode,name,type,value);
	guard gl(worker->innerModel->mutex);

	QString qIdNode=QString::fromStdString(idNode);
	QString qName=QString::fromStdString(name);
	QString m="RoboCompInnerModelManager::getAttribute()";

	InnerModelNode *node = worker->getNode(qIdNode, m);
	worker->NonExistingAttribute(node, qName,m);

	type= node->attributes[qName].type.toStdString();
	value=node->attributes[qName].value.toStdString();

	return true;
}


void InnerModelManagerI::getAllNodeInformation(NodeInformationSequence &nodesInfo, const Ice::Current&)
{
	guard gl(worker->innerModel->mutex);
	nodesInfo.clear();
	worker->getRecursiveNodeInformation(nodesInfo, worker->innerModel->getRoot());
}

//void InnerModelManagerI::addPointCloud(const std::string &id, const Ice::Current&)
//{
// 	worker->addPointCloud(id);
//

void InnerModelManagerI::setPointCloudData(const std::string &idNode, const PointCloudVector &cloud, const Ice::Current&)
{
	//worker->imm_setPointCloudData(id, idNode, cloud);
	guard gl(worker->innerModel->mutex);
	QString m = QString("SpecificWorker::setPointCloudData");
	std::cout << "setPointCloudData: " << idNode << " " << cloud.size() <<std::endl;

	/// Aqui Marco va a mejorar el código :-) felicidad (comprobar que la nube existe)
	IMVPointCloud *pcNode = worker->imv->pointCloudsHash[QString::fromStdString(idNode)];

	int points = cloud.size();
	pcNode->points->resize(points);
	pcNode->colors->resize(points);
	pcNode->setPointSize(1);
	for(int i=0 ; i<points; i++) 
	{
		pcNode->points->operator[](i) = QVecToOSGVec(QVec::vec3(cloud[i].x, cloud[i].y, cloud[i].z));
		pcNode->colors->operator[](i) = osg::Vec4f(float(cloud[i].r) /255, float(cloud[i].g) /255, float(cloud[i].b) /255, 1.f);
	}
	pcNode->update();
}

bool InnerModelManagerI::collide(const std::string &a, const std::string &b, const Ice::Current&)
{
	//return worker->imm_collide(a, b);
	try
	{
		guard gl(worker->innerModel->mutex);
		return 	worker->innerModel->collide(QString::fromStdString(a), QString::fromStdString(b));
	}
	catch (int err)
	{
		RoboCompInnerModelManager::InnerModelManagerError ex;
		ex.err = RoboCompInnerModelManager::NonExistingNode;
		std::ostringstream oss;
		oss << "InnerModelManager::collide: Error, cannot find node ";
		if (err == 1)
		{
			oss << a;
		}
		else
		{
			oss << b;
		}
		ex.text = oss.str();
		printf("ERROR: %s\n", ex.text.c_str());
		throw ex;
	}
}




