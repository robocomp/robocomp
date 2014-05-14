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
#ifndef INNERMODELMANAGERI_H
#define INNERMODELMANAGERI_H

// Qt includes
#include <QMutex>
#include <QObject>
#include <QThread>

// RoboComp includes
#include <Ice/Ice.h>
#include <InnerModelManager.h>
#include <innermodel/innermodel.h>
#include <string>

// Simulator includes
#include "config.h"



using namespace RoboCompInnerModelManager;

class SpecificWorker;

class InnerModelManagerI : public QThread, public virtual RoboCompInnerModelManager::InnerModelManager
{
	Q_OBJECT
public:
	InnerModelManagerI ( SpecificWorker *w );
	~InnerModelManagerI() {};
	
	bool setPose ( const std::string& item, const std::string& base, const Pose3D& pose, const Ice::Current& );
	bool setPoseFromParent ( const std::string& item, const Pose3D& pose, const Ice::Current& );
	bool getPose ( const std::string& item, const std::string& base, Pose3D& pose, const Ice::Current& );
	bool getPoseFromParent ( const std::string& item, Pose3D& pose, const Ice::Current& );
	
	bool transform ( const std::string& item, const std::string& base, const coord3D& coordInItem, coord3D& coordInBase, const Ice::Current& );
	Matrix getTransformationMatrix ( const std::string& item, const std::string& base, const Ice::Current& );
	bool setScale ( const std::string& item, float scaleX,float scaleY, float scaleZ, const Ice::Current& );
	
	bool setPlane ( const std::string& item, const Plane3D& pose, const Ice::Current& );
	bool addTransform ( const std::string& item, const std::string& engine, const std::string& base, const Pose3D& pose, const Ice::Current& );
	bool addJoint ( const std::string& item,const std::string& base, const jointType& j, const Ice::Current& );
	bool addMesh ( const std::string& item, const std::string& base, const meshType& m, const Ice::Current& );
	bool addPlane ( const std::string &item, const std::string &base, const Plane3D &p, const Ice::Current& );
	
	bool addAttribute ( const std::string& idNode, const std::string& name, const std::string& type, const std::string& value, const Ice::Current& );
	bool removeAttribute ( const std::string& idNode, const std::string& name, const Ice::Current& );
	bool setAttribute ( const std::string& idNode, const std::string& name, const std::string& type, const std::string& value,const Ice::Current& );
	bool getAttribute ( const std::string& idNode, const std::string& name, std::string& type, std::string& value, const Ice::Current& );
	
	bool removeNode ( const std::string& item, const Ice::Current& );
	
	void getAllNodeInformation ( NodeInformationSequence &nodesInfo, const Ice::Current& );
	
	void setPointCloudData ( const std::string &id, const RoboCompInnerModelManager::PointCloudVector &cloud, const Ice::Current& );

	bool collide(const std::string &a, const std::string &b, const Ice::Current&);
private:
	SpecificWorker *worker;
	QString id;
};

#endif
