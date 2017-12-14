#include "specificworker.h"

// ------------------------------------------------------------------------------------------------
// InnerModelManager.ice
// ------------------------------------------------------------------------------------------------
//#define INNERMODELMANAGERDEBUG

// Moves item to the position defined by pose respect to the base
bool SpecificWorker::imm_setPose(const QString &server, const std::string &base, const std::string &item, const RoboCompInnerModelManager::Pose3D &pose)
{
	QMutexLocker locker(mutex);

	QString qBase = QString::fromStdString(base);
	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::setPose()";

	//check type transform
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(getNode(qBase, m));
	checkOperationInvalidNode(aux, m + qBase +"can't be use as base because it's not a InnerModelTransform node.");
	aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(getNode(qItem, m));
	checkOperationInvalidNode(aux, m + qItem +"can't be use as item because it's not a InnerModelTransform node.");

	innerModel->updateTransformValues(qItem, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, qBase);

	return true;
}


bool SpecificWorker::imm_setPoseFromParent(const QString &server, const std::string &item, const RoboCompInnerModelManager::Pose3D &pose)
{
	QMutexLocker locker(mutex);

	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::setPose()";

	//check type transform
	InnerModelTransform *aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(getNode(qItem, m));
	checkOperationInvalidNode(aux, m + qItem +"can't be use as item because it's not a InnerModelTransform node.");

	innerModel->updateTransformValues(qItem, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);

	return true;
}


/// ---------------------------------------------------------------------------------------
// Provides the pose of a certain item respect to the base.
/// ---------------------------------------------------------------------------------------
bool SpecificWorker::imm_getPose(const QString &server, const std::string &base, const std::string &item, RoboCompInnerModelManager::Pose3D &pose)
{
	QMutexLocker locker(mutex);
	QVec p;
	QString qBase = QString::fromStdString(base);
	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::getPose()";

	// check type transform
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(getNode(qBase, m));
	checkOperationInvalidNode(aux, m + qBase +"can't be use as base because it's not a InnerModelTransform node.");
	aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(getNode(qItem, m));
	checkOperationInvalidNode(aux, m + qItem +"can't be use as item because it's not a InnerModelTransform node.");

	// calculate position
	p = innerModel->transform(QString::fromUtf8(base.c_str()), QVec::vec3(0,0,0), QString::fromUtf8(item.c_str()));
	pose.x = p.x();
	pose.y = p.y();
	pose.z = p.z();
	//calulate rotation
	p = innerModel->getRotationMatrixTo(QString::fromUtf8(base.c_str()), QString::fromUtf8(item.c_str())).extractAnglesR();
	pose.rx = p.x();
	pose.ry = p.y();
	pose.rz = p.z();

	return true;
}


// Provides the pose of a certain item respect to the parent
bool SpecificWorker::imm_getPoseFromParent(const QString &server, const std::string &item, RoboCompInnerModelManager::Pose3D &pose)
{
	QMutexLocker locker(mutex);
	QString m="RoboCompInnerModelManager::getPoseFromParent()";

	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(getNode(QString::fromStdString(item), m));
	checkOperationInvalidNode(aux, m+aux->id+"can't be use as base because it's not a InnerModelTransform node.");

	pose.x = aux->backtX;
	pose.y = aux->backtY;
	pose.z = aux->backtZ;
	pose.rx = aux->backrX;
	pose.ry = aux->backrY;
	pose.rz = aux->backrZ;

	return true;
}


// Provides the transform of a certain point expressed in Base to Item.
bool SpecificWorker::imm_transform(const QString &server, const std::string &base, const std::string &item, const RoboCompInnerModelManager::coord3D &coordInItem, RoboCompInnerModelManager::coord3D &coordInBase)
{
	QMutexLocker locker(mutex);
	QVec p;
	const QString qBase = QString::fromStdString(base);
	const QString qItem = QString::fromStdString(item);
	const QString m="RoboCompInnerModelManager::transform()";

	//check type transform
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(getNode(qBase, m));
	checkOperationInvalidNode(aux, m + qBase +"can't be used as base because it's not a InnerModelTransform node.");

	aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(getNode(qItem, m));
	checkOperationInvalidNode(aux, m + qItem +"can't be used as item because it's not a InnerModelTransform node.");

	// 	QString qBase = QString::fromStdString(base);
// 	QString qItem = QString::fromStdString(item);
// 	QString m="RoboCompInnerModelManager::transform()";
//
// 	//check type transform
// 	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(getNode(qBase, m));
// 	checkOperationInvalidNode(aux, m + qBase +"can't be use as base because it's not a InnerModelTransform node.");
//
// 	aux = NULL;
// 	aux = dynamic_cast<InnerModelTransform*>(getNode(qItem, m));
// 	checkOperationInvalidNode(aux, m + qItem +"can't be use as item because it's not a InnerModelTransform node.");

	// calculate position
	p = innerModel->transform(QString::fromUtf8(base.c_str()), QVec::vec3(coordInItem.x,coordInItem.y,coordInItem.z),QString::fromUtf8(item.c_str()));
	coordInBase.x = p.x();
	coordInBase.y = p.y();
	coordInBase.z = p.z();

	return true;
}

RoboCompInnerModelManager::Matrix SpecificWorker::imm_getTransformationMatrix(const std::string &item, const std::string &base)
{
// 	qFatal("Not well implemented yet.");
	const QString qBase = QString::fromStdString(base);
	const QString qItem = QString::fromStdString(item);
	const QString m="RoboCompInnerModelManager::transform()";
	QMutexLocker locker(mutex);

	//check type transform
	InnerModelTransform *aux = dynamic_cast<InnerModelTransform*>(getNode(qBase, m));
	checkOperationInvalidNode(aux, m + qBase +"can't be used as base because it's not a InnerModelTransform node.");

	aux = NULL;
	aux = dynamic_cast<InnerModelTransform*>(getNode(qItem, m));
	checkOperationInvalidNode(aux, m + qItem +"can't be used as item because it's not a InnerModelTransform node.");

	// calculate position
	RTMat retA = innerModel->getTransformationMatrix(qBase, qItem);
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


// Scales a mesh to a new size.
bool SpecificWorker::imm_setScale(const QString &server, const std::string &item, float scaleX, float scaleY, float scaleZ)
{
// 	QMutexLocker locker(mutex);
// 	QString qItem = QString::fromStdString(item);
// 	QString m="RoboCompInnerModelManager::setScale()";
//
// 	InnerModelMesh *aux = dynamic_cast<InnerModelMesh*>(getNode(QString::fromStdString(item),m));
// 	checkOperationInvalidNode(aux, m + qItem +"can't be used because it's not a InnerModelMesh node.");
//
// 	aux->setScale(scaleX, scaleY, scaleZ);
// 	innerModel->update();
//
// #ifdef INNERMODELMANAGERDEBUG
// 	try {
// 		checkPoseCollision(qItem,m);
// 	} catch(RoboCompInnerModelManager::InnerModelManagerError err) {
// 		std::cout<<err.what() <<" "<<err.text<< "\n";
// 		std::cout<< "\n";
// 		///come back to t= (t+1) -t
//
// 		//to check => maybe using a tag in the xml (collide="true") to decide if allow collitions or not
// 		//  innerModel->updateTransformValues(qItem,p.x, p.y, p.z, p.rx , p.ry, p.rz);
// 		//  innerModel->update();
// 		throw err;
// 	}
// #endif

	QMutexLocker locker(mutex);
	QString qItem = QString::fromStdString(item);
	QString m="RoboCompInnerModelManager::setScale()";

	InnerModelMesh *aux = dynamic_cast<InnerModelMesh*>(getNode(QString::fromStdString(item),m));
	checkOperationInvalidNode(aux,m + qItem +"can't be used because it's not a InnerModelMesh node.");

	aux->setScale(scaleX, scaleY, scaleZ);
	//I think not necessary
	// 	innerModel->update();

#ifdef INNERMODELMANAGERDEBUG
// 	try {
// 		checkPoseCollision(qItem,m);
// 	} catch(RoboCompInnerModelManager::InnerModelManagerError err) {
// 		std::cout<<err.what() <<" "<<err.text<< "\n";
// 		std::cout<< "\n";
// 		///come back to t= (t+1) -t
//
// 		//to check => maybe using a tag in the xml (collide="true") to decide if allow collitions or not
// 		//  innerModel->updateTransformValues(qItem,p.x, p.y, p.z, p.rx , p.ry, p.rz);
// 		//  innerModel->update();
// 		throw err;
// 	}
#endif

	return true;
}


bool SpecificWorker::imm_setPlane(const QString &server, const std::string &item, const RoboCompInnerModelManager::Plane3D &plane)
{
// 	QMutexLocker locker(mutex);
// 	QString m="RoboCompInnerModelManager::setPlane()";
// 	printf("SETPLANE %s: %f_%f_%f\n", item.c_str(), plane.px, plane.py, plane.pz);
// 	printf("SETPLANE %s: %f_%f_%f\n", item.c_str(), plane.px, plane.py, plane.pz);
// 	printf("SETPLANE %s: %f_%f_%f\n", item.c_str(), plane.px, plane.py, plane.pz);
// 	printf("SETPLANE %s: %f_%f_%f\n", item.c_str(), plane.px, plane.py, plane.pz);
// 	InnerModelPlane *aux = dynamic_cast<InnerModelPlane*>(getNode(QString::fromStdString(item), m));
// 	checkOperationInvalidNode(aux, m + aux->id +"can't be use as base because it's not of the type InnerModelPlane.");
// 	innerModel->updatePlaneValues(QString::fromStdString(item), plane.nx, plane.ny, plane.nz, plane.px, plane.py, plane.pz);
//
// 	return true;

	QMutexLocker locker(mutex);
	QString m="RoboCompInnerModelManager::setPlane()";
// 	printf("SETPLANE %s: %f_%f_%f\n", item.c_str(), plane.px, plane.py, plane.pz);
	InnerModelPlane *aux = dynamic_cast<InnerModelPlane*>(getNode(QString::fromStdString(item), m));
	checkOperationInvalidNode(aux,m + aux->id +"can't be use as base because it's not of the type InnerModelPlane.");
	innerModel->updatePlaneValues(QString::fromStdString(item), plane.nx, plane.ny, plane.nz, plane.px, plane.py, plane.pz);
	return true;
}

bool SpecificWorker::imm_setPlaneTexture(const QString &server, const std::string &item, const std::string &texture)
{

	QMutexLocker locker(mutex);
	QString m="RoboCompInnerModelManager::setPlaneTexture()";
	printf("SETPLANETEXTURE %s: %s\n", item.c_str(), texture.c_str());
	InnerModelPlane *aux = dynamic_cast<InnerModelPlane*>(getNode(QString::fromStdString(item), m));
	qDebug()<<"aux->texture"<<aux->texture<<"qstring"<<QString::fromStdString(texture);

	aux->texture=QString::fromStdString(texture);

	osg::Image *image=NULL;
	image = osgDB::readImageFile(texture);
	if (not image)
	{
		qDebug() << "Couldn't load texture:" << texture.c_str();
		throw "Couldn't load texture.";
	}

	imv->planesHash[aux->id]->image =image;
	imv->planesHash[aux->id]->texture->setImage(image);

	qDebug()<<"change aux->texture"<<aux->texture;
// 	checkOperationInvalidNode(aux,m + aux->id +"can't be use as base because it's not of the type InnerModelPlane.");
// 	innerModel->updatePlaneValues(QString::fromStdString(item), plane.nx, plane.ny, plane.nz, plane.px, plane.py, plane.pz);
	return true;
}



// void SpecificWorker::addPointCloud(const std::string &id)
// {
// }


void SpecificWorker::imm_setPointCloudData(const QString &server, const std::string &id, const RoboCompInnerModelManager::PointCloudVector &cloud)
{
// 	QString m = QString("SpecificWorker::setPointCloudData");
//
// 	std::cout<<"setPointCloudData: "<<id<<" "<<cloud.size() <<std::endl;
//
// 	/// Aqui Marco va a mejorar el código :-) felicidad (comprobar que la nube existe)
// 	IMVPointCloud *pcNode = imv->pointCloudsHash[QString::fromStdString(id)];
//
// 	int points = cloud.size();
// 	pcNode->points->resize(points);
// 	pcNode->colors->resize(points);
// 	pcNode->setPointSize(1);
// 	for(int i=0 ; i<points; i++) {
// 		pcNode->points->operator[](i) = QVecToOSGVec(QVec::vec3(cloud[i].x, -cloud[i].y, cloud[i].z));
// 		pcNode->colors->operator[](i) = osg::Vec4f(float(cloud[i].r) /255, float(cloud[i].g) /255, float(cloud[i].b) /255, 1.f);
// 	}
// 	pcNode->update();
// 	imv->update();

	QString m = QString("SpecificWorker::setPointCloudData");

	std::cout<<"setPointCloudData: "<<id<<" "<<cloud.size() <<std::endl;

	/// Aqui Marco va a mejorar el código :-) felicidad (comprobar que la nube existe)
	IMVPointCloud *pcNode = imv->pointCloudsHash[QString::fromStdString(id)];

	int points = cloud.size();
	pcNode->points->resize(points);
	pcNode->colors->resize(points);
	pcNode->setPointSize(1);
	for(int i=0 ; i<points; i++) {
		pcNode->points->operator[](i) = QVecToOSGVec(QVec::vec3(cloud[i].x, cloud[i].y, cloud[i].z));
		pcNode->colors->operator[](i) = osg::Vec4f(float(cloud[i].r) /255, float(cloud[i].g) /255, float(cloud[i].b) /255, 1.f);
	}
	pcNode->update();
//I think not necessary
// 	imv->update();
}


bool SpecificWorker::imm_addTransform(const QString &server, const std::string &item, const std::string &engine, const std::string &base, const RoboCompInnerModelManager::Pose3D &pose)
{
	QMutexLocker locker(mutex);
	InnerModelNode *parent = getNode(QString::fromStdString(base), "RoboCompInnerModelManager::addTransform()");
	checkNodeAlreadyExists(QString::fromStdString(item), "RoboCompInnerModelManager::addTransform()");

	QString qEngine = QString::fromStdString( engine);
	if (qEngine !="static" and qEngine !="bullet")
	{
		qEngine = "static";
	}

	InnerModelTransform *tr = innerModel->newTransform(QString::fromStdString(item), QString::fromStdString("static") ,parent, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
	parent->addChild(tr);
	imv->recursiveConstructor(tr, imv->mts[parent->id], imv->mts, imv->meshHash); // imv->osgmeshes,imv->osgmeshPats);

#ifdef INNERMODELMANAGERDEBUG
	qDebug()<<"engine"<<qEngine;
	qDebug() <<"transform: pose.x<<pose.y<<pose.z"<<pose.x<<pose.y<<pose.z<<QString::fromStdString(item);
#endif
	return true;
}


bool SpecificWorker::imm_addJoint(const QString &server, const std::string &item, const std::string &base, const RoboCompInnerModelManager::jointType &j_)
{
	RoboCompInnerModelManager::jointType j = j_;
	if (j.axis == "")
	{
		j.axis = "z";
	}

	QMutexLocker locker(mutex);
	RoboCompInnerModelManager::Pose3D pose = j.pose;

	InnerModelTransform *parent=dynamic_cast<InnerModelTransform *>(getNode(QString::fromStdString(base), "RoboCompInnerModelManager::addJoint()"));
	checkNodeAlreadyExists(QString::fromStdString(item), "RoboCompInnerModelManager::addJoint()");


	InnerModelJoint *jN = innerModel->newJoint(QString::fromStdString(item), parent, j.lx, j.ly, j.lz, j.hx, j.hy, j.hz, pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, j.min, j.max, j.port, j.axis);
	parent->addChild (jN);

	// Create Interface in case the port is not 0
	if (jN->port != 0)
		addJM(jN);

	imv->recursiveConstructor(jN, imv->mts[parent->id], imv->mts, imv->meshHash); // imv->osgmeshes,imv->osgmeshPats);

	return true;
}


bool SpecificWorker::imm_addMesh(const QString &server, const std::string &item, const std::string &base, const RoboCompInnerModelManager::meshType &m)
{
	QMutexLocker locker(mutex);
	QString msg="RoboCompInnerModelManager::addMesh()";
#ifdef INNERMODELMANAGERDEBUG
	qDebug() <<msg<<QString::fromStdString(base) <<QString::fromStdString(item);
	qDebug() <<QString::fromStdString(m.meshPath);
#endif
	InnerModelTransform *parent = dynamic_cast<InnerModelTransform*>(getNode(QString::fromStdString(base), msg));

	//Checking if its parent is not a mesh.
	checkOperationInvalidNode(parent, msg);
	checkNodeAlreadyExists(QString::fromStdString(item), msg);
	checkInvalidMeshValues(m,msg);

	int render = m.render;
	if(render!=0 and render!=1)
	{
		render=0;
	}

	InnerModelMesh *mesh = innerModel->newMesh (
		QString::fromStdString(item),
		parent,
		QString::fromStdString(m.meshPath),
		m.scaleX, m.scaleY, m.scaleZ,
		render,
		m.pose.x, m.pose.y, m.pose.z,
		m.pose.rx, m.pose.ry, m.pose.rz);

	mesh->setScale(m.scaleX, m.scaleY, m.scaleZ);
	parent->addChild(mesh);

	imv->recursiveConstructor(mesh, imv->mts[parent->id], imv->mts, imv->meshHash); // osgmeshes,imv->osgmeshPats);
	//I think not necessary
	//imv->update();

	///create boundingBox as innerModelNodeMesh when it has been added to innermodel and imnnermodelViewer
	///TODO create boundingBox
	///Create rapid model for the new mesh
// 	if(collisiondetection->isChecked()) {
// 		meshColision[QString::fromStdString(item)]=cargaTris(QString::fromStdString(item));
// 	}

	return true;
}


bool SpecificWorker::imm_addPlane(const QString &server, const std::string &item, const std::string &base, const RoboCompInnerModelManager::Plane3D &p)
{
// 	QMutexLocker locker(mutex);
//
// 	InnerModelNode *parent = getNode(QString::fromStdString(base), "RoboCompInnerModelManager::addPlane()");
// 	checkNodeAlreadyExists(QString::fromStdString(item), "RoboCompInnerModelManager::addPlane()");
//
// 	printf("ADDPLANE %s: %f_%f_%f\n", item.c_str(), p.px, p.py, p.pz);
// 	printf("ADDPLANE %s: %f_%f_%f\n", item.c_str(), p.px, p.py, p.pz);
// 	printf("ADDPLANE %s: %f_%f_%f\n", item.c_str(), p.px, p.py, p.pz);
// 	printf("ADDPLANE %s: %f_%f_%f\n", item.c_str(), p.px, p.py, p.pz);
// 	printf("ADDPLANE %s: %f_%f_%f\n", item.c_str(), p.px, p.py, p.pz);
// 	printf("ADDPLANE %s: %f_%f_%f\n", item.c_str(), p.px, p.py, p.pz);
// 	printf("ADDPLANE %s: %f_%f_%f\n", item.c_str(), p.px, p.py, p.pz);
//
// 	InnerModelPlane *plane = innerModel->newPlane(QString::fromStdString(item), parent, QString::fromStdString(p.texture),
// 	                         p.width, p.height, p.thickness, 1,
// 	                         p.nx, p.ny, p.nz, p.px, p.py, p.pz);
// 	parent->addChild(plane);
//
// 	imv->recursiveConstructor(plane, imv->mts[parent->id], imv->mts, imv->meshHash);
// 	imv->update();

	QMutexLocker locker(mutex);

	InnerModelNode *parent = getNode(QString::fromStdString(base), "RoboCompInnerModelManager::addPlane()");
	checkNodeAlreadyExists(QString::fromStdString(item), "RoboCompInnerModelManager::addPlane()");


	InnerModelPlane *plane = innerModel->newPlane(QString::fromStdString(item), parent, QString::fromStdString(p.texture),
	                         p.width, p.height, p.thickness, 1,
	                         p.nx, p.ny, p.nz, p.px, p.py, p.pz);
	parent->addChild(plane);

	imv->recursiveConstructor(plane, imv->mts[parent->id], imv->mts, imv->meshHash);
	//I think not necessary
	// 	imv->update();

	return true;
}


bool SpecificWorker::imm_addAttribute(const QString &server, const std::string &idNode, const std::string &name, const std::string &type, const std::string &value)
{
	QMutexLocker locker(mutex);
	QString qIdNode=QString::fromStdString(idNode);
	QString qName=QString::fromStdString(name);
	QString qType=QString::fromStdString(type);
	QString qValue=QString::fromStdString(value);
	QString m="RoboCompInnerModelManager::addAttribute()";

	InnerModelNode *node = getNode(qIdNode, m);
	AttributeAlreadyExists(node,qName,m);

	InnerModelNode::AttributeType t;
// 	InnerModelNode::
// 	AttributeType t;
	t.type=qType;
	t.value=qValue;
	node->attributes.insert(qName,t);

	return true;
}


bool SpecificWorker::imm_setAttribute(const QString &server, const std::string &idNode, const std::string &name, const std::string &type, const std::string &value)
{
	QMutexLocker locker(mutex);
	QString qIdNode=QString::fromStdString(idNode);
	QString qName=QString::fromStdString(name);
	QString qType=QString::fromStdString(type);
	QString qValue=QString::fromStdString(value);

	QString m="RoboCompInnerModelManager::setAttribute()";

	InnerModelNode *node = getNode(qIdNode, m);
	NonExistingAttribute(node,qName,m);

	node->attributes[qName].type=qType;
	node->attributes[qName].value=qValue;

	return true;
}


bool SpecificWorker::imm_getAttribute(const QString &server, const std::string &idNode, const std::string &name, std::string &type, std::string &value)
{
	QMutexLocker locker(mutex);
	QString qIdNode=QString::fromStdString(idNode);
	QString qName=QString::fromStdString(name);
	QString m="RoboCompInnerModelManager::getAttribute()";

	InnerModelNode *node = getNode(qIdNode, m);
	NonExistingAttribute(node, qName,m);

	type= node->attributes[qName].type.toStdString();
	value=node->attributes[qName].value.toStdString();

	return true;
}


bool SpecificWorker::imm_removeAttribute(const QString &server, const std::string &idNode, const std::string &name)
{
	QMutexLocker locker(mutex);
	QString qIdNode=QString::fromStdString(idNode);
	QString qName=QString::fromStdString(name);
	QString m="RoboCompInnerModelManager::removeAttribute()";

	InnerModelNode *node = getNode(qIdNode, m);
	NonExistingAttribute(node, qName,m);

	node->attributes.remove(qName);

	return true;
}


bool SpecificWorker::imm_removeNode(const QString &server, const std::string &item)
{
		QMutexLocker locker(mutex);
	QString msg="RoboCompInnerModelManager::removeNode()";
#ifdef INNERMODELMANAGERDEBUG
	qDebug() <<msg<<QString::fromStdString(item);
#endif

	QString id =QString::fromStdString(item);
	if(id=="world" || id=="root") {
#ifdef INNERMODELMANAGERDEBUG
		qDebug() <<msg<<id<<"Can't remove root elements";
#endif
		RoboCompInnerModelManager::InnerModelManagerError err;
		err.err = RoboCompInnerModelManager::OperationInvalidNode;
		std::ostringstream oss;
		oss <<msg.toStdString() <<" error, cannot remove. Node: " <<id.toStdString();
		err.text = oss.str();
		throw err;
	}

	InnerModelNode *node = getNode(QString::fromStdString(item), msg);
	checkOperationInvalidNode(node,msg);

	QStringList l;
	l.clear();

	innerModel->getSubTree(node,&l);
// 	qDebug()<<"----------- l.size()"<<l.size();
	///remove handlers and node
	foreach (QString n, l)
	{
		///remove handlers
#ifdef INNERMODELMANAGERDEBUG
 		qDebug()<<"remove"<<n;
#endif
		InnerModelJoint *jN = dynamic_cast<InnerModelJoint *> (innerModel->getNode(n));
		if (jN!=NULL && jN->port!=0)
		{
#ifdef INNERMODELMANAGERDEBUG
 			qDebug()<<"remove Joint"<<n<<jN->port;
#endif
			removeJM(jN);

		}

		///remove nodes in InnerModel tree
// 		innerModel->removeNode(n);
	}
	innerModel->removeSubTree(node,&l);


	/// Replicate InnerModel node removals in the InnerModelViewer tree. And in handlers Lists
	foreach(QString n, l) {
		/// Replicate plane removals
		if(imv->meshHash.contains(n)) {
// 			qDebug()<<"/// Replicate meshHash removals"<<n;
			while(imv->meshHash[n].osgmeshPaths->getNumParents() > 0)
				( imv->meshHash[n].osgmeshPaths->getParent(0))->removeChild(imv->meshHash[n].osgmeshPaths);
			while(imv->meshHash[n].osgmeshes->getNumParents() > 0)
				( imv->meshHash[n].osgmeshes->getParent(0))->removeChild(imv->meshHash[n].osgmeshes);
			while(imv->meshHash[n].meshMts->getNumParents() > 0)
				( imv->meshHash[n].meshMts->getParent(0))->removeChild(imv->meshHash[n].meshMts);

			imv->meshHash.remove(n);
// 			meshColision.remove(n);
		}
		/// Replicate transform removals
		if(imv->mts.contains(n)) {
//			qDebug()<<"/// Replicate transform removals";//<<n<<imv->mts[n]->getNumParents();
 			while(imv->mts[n]->getNumParents() > 0) {
				(imv->mts[n]->getParent(0))->removeChild(imv->mts[n]);
 			}
 			imv->mts.remove(n);
		}
		/// Replicate plane removals
		if(imv->planeMts.contains(n)) {
//			qDebug()<<"/// Replicate plane removals";
			while(imv->planeMts[n]->getNumParents() > 0) {
				((osg::Group *)(imv->planeMts[n]->getParent(0)))->removeChild(imv->planeMts[n]);
			}
			imv->planeMts.remove(n);
			imv->planesHash.remove(n);
		}

	}
// 	qDebug()<<imv->meshHash.size();
// 	qDebug()<<imv->mts.size();
// 	qDebug()<<imv->planeMts.size();
// 	qDebug()<<imv->planesHash.size();
// 	qDebug()<<innerModel->getIDKeys().size();
// 	innerModel->print();
// 	qDebug()<<"----";
//I think not necessary
// 	innerModel->update();
// 	imv->update();

	return true;
}
#define INNERMODELMANAGERDEBUG
bool SpecificWorker::imm_moveNode(const QString &server, const std::string &src, const std::string &dst)
{
	QMutexLocker locker(mutex);
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


	InnerModelNode *nodeSrc = getNode(idSrc, msg);
	checkOperationInvalidNode(nodeSrc,msg);

	InnerModelNode *nodeDst = getNode(idDst, msg);
	checkOperationInvalidNode(nodeDst,msg);

	QStringList l;
	l.clear();

	//consigo ids para viewer
	innerModel->getSubTree (nodeSrc,&l);
	//muevo
	innerModel->moveSubTree(nodeSrc,nodeDst);




	/// Replicate InnerModel node removals in the InnerModelViewer tree. And in handlers Lists
	foreach(QString n, l) {
		/// Replicate plane removals
		if(imv->meshHash.contains(n)) {
// 			qDebug()<<"/// Replicate meshHash removals"<<n;
			while(imv->meshHash[n].osgmeshPaths->getNumParents() > 0)
				( imv->meshHash[n].osgmeshPaths->getParent(0))->removeChild(imv->meshHash[n].osgmeshPaths);
			while(imv->meshHash[n].osgmeshes->getNumParents() > 0)
				( imv->meshHash[n].osgmeshes->getParent(0))->removeChild(imv->meshHash[n].osgmeshes);
			while(imv->meshHash[n].meshMts->getNumParents() > 0)
				( imv->meshHash[n].meshMts->getParent(0))->removeChild(imv->meshHash[n].meshMts);

			imv->meshHash.remove(n);
// 			meshColision.remove(n);
		}
		/// Replicate transform removals
		if(imv->mts.contains(n)) {
//			qDebug()<<"/// Replicate transform removals";//<<n<<imv->mts[n]->getNumParents();
 			while(imv->mts[n]->getNumParents() > 0) {
				(imv->mts[n]->getParent(0))->removeChild(imv->mts[n]);
 			}
 			imv->mts.remove(n);
		}
		/// Replicate plane removals
		if(imv->planeMts.contains(n)) {
//			qDebug()<<"/// Replicate plane removals";
			while(imv->planeMts[n]->getNumParents() > 0) {
				((osg::Group *)(imv->planeMts[n]->getParent(0)))->removeChild(imv->planeMts[n]);
			}
			imv->planeMts.remove(n);
			imv->planesHash.remove(n);
		}

	}
	foreach(QString n, l)
	{

		imv->recursiveConstructor(innerModel->getNode(n), imv->mts[innerModel->getNode(n)->parent->id], imv->mts, imv->meshHash); // imv->osgmeshes,imv->osgmeshPats);
	}


// 	qDebug()<<imv->meshHash.size();
// 	qDebug()<<imv->mts.size();
// 	qDebug()<<imv->planeMts.size();
// 	qDebug()<<imv->planesHash.size();
// 	qDebug()<<innerModel->getIDKeys().size();
// 	innerModel->print();
	qDebug()<<"-- fin move Subtree --";
	return true;
}


void SpecificWorker::imm_getAllNodeInformation(const QString &server, RoboCompInnerModelManager::NodeInformationSequence &nodesInfo)
{
	nodesInfo.clear();
	getRecursiveNodeInformation(nodesInfo, innerModel->getRoot());
}


bool SpecificWorker::imm_collide(const string &a, const string &b)
{
	try
	{
		return innerModel->collide(QString::fromStdString(a), QString::fromStdString(b));
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
