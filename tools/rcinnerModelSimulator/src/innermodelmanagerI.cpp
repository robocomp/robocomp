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


InnerModelManagerI::InnerModelManagerI(SpecificWorker *w)
{
	worker = w;
}


bool InnerModelManagerI::setPose(const std::string &base, const std::string &item, const RoboCompInnerModelManager::Pose3D &pose, const Ice::Current&)
{
	return worker->imm_setPose(id, base, item, pose);
}


bool InnerModelManagerI::setPoseFromParent(const std::string &item, const RoboCompInnerModelManager::Pose3D &pose, const Ice::Current&)
{
	return worker->imm_setPoseFromParent(id, item, pose);
}


bool InnerModelManagerI::getPose(const std::string &base, const std::string &item, RoboCompInnerModelManager::Pose3D &pose, const Ice::Current&)
{
	return worker->imm_getPose(id, base, item, pose);
}


bool InnerModelManagerI::getPoseFromParent(const std::string &item, RoboCompInnerModelManager::Pose3D &pose, const Ice::Current&)
{
	return worker->imm_getPoseFromParent(id, item, pose);
}


bool InnerModelManagerI::transform(const std::string &base, const std::string &item, const coord3D &coordInItem, coord3D &coordInBase, const Ice::Current&)
{
	return worker->imm_transform(id, base, item, coordInItem, coordInBase);
}

Matrix InnerModelManagerI::getTransformationMatrix(const std::string &base, const std::string &item, const Ice::Current&)
{
	return worker->imm_getTransformationMatrix(base, item);
}


bool InnerModelManagerI::setScale(const std::string &item, float scaleX,float scaleY, float scaleZ, const Ice::Current&)
{
	return worker->imm_setScale(id, item, scaleX, scaleY, scaleZ);
}


bool InnerModelManagerI::setPlane(const std::string &item, const RoboCompInnerModelManager::Plane3D &plane, const Ice::Current&)
{
	return worker->imm_setPlane(id, item, plane);
}
bool InnerModelManagerI::setPlaneTexture(const std::string &item, const std::string &texture, const Ice::Current&)
{
	return worker->imm_setPlaneTexture(id, item, texture);
}


bool InnerModelManagerI::addTransform(const std::string &item, const std::string &engine, const std::string &base,const Pose3D &pose, const Ice::Current&)
{
	return worker->imm_addTransform(id, item, engine, base, pose);
}


bool InnerModelManagerI::addJoint(const std::string &item,const std::string &base, const jointType &j, const Ice::Current&)
{
	return worker->imm_addJoint(id, item, base, j);
}


bool InnerModelManagerI::addMesh(const std::string &item,const std::string &base,const meshType &m, const Ice::Current&)
{
	return worker->imm_addMesh(id, item, base, m);
}


bool InnerModelManagerI::addPlane(const std::string &item, const std::string &base, const Plane3D &p, const Ice::Current&)
{
	return worker->imm_addPlane(id, item, base, p);
}


bool InnerModelManagerI::removeNode(const std::string &item, const Ice::Current&)
{
	return worker->imm_removeNode(id, item);
	
}

bool InnerModelManagerI::moveNode(const std::string &src, const std::string &dst, const Ice::Current&)
{
	return worker->imm_moveNode(id, src,dst);
	
}


bool InnerModelManagerI::addAttribute(const string &idNode, const string &name, const string &type, const string &value, const Ice::Current&)
{
	return worker->imm_addAttribute(id, idNode, name, type, value);
}


bool InnerModelManagerI::removeAttribute(const string &idNode, const string &name, const Ice::Current&)
{
	return worker->imm_removeAttribute(id, idNode, name);
}


bool InnerModelManagerI::setAttribute(const string &idNode, const string &name, const string &type, const string &value, const Ice::Current&)
{
	return worker->imm_setAttribute(id, idNode, name, type, value);
}


bool InnerModelManagerI::getAttribute(const string &idNode, const string &name,  string &type,  string &value, const Ice::Current&)
{
	return worker->imm_getAttribute(id, idNode,name,type,value);
}


void InnerModelManagerI::getAllNodeInformation(NodeInformationSequence &nodesInfo, const Ice::Current&)
{
	worker->imm_getAllNodeInformation(id, nodesInfo);
}

//void InnerModelManagerI::addPointCloud(const std::string &id, const Ice::Current&)
//{
// 	worker->addPointCloud(id);
//

void InnerModelManagerI::setPointCloudData(const std::string &idNode, const PointCloudVector &cloud, const Ice::Current&)
{
	worker->imm_setPointCloudData(id, idNode, cloud);
}

bool InnerModelManagerI::collide(const std::string &a, const std::string &b, const Ice::Current&)
{
	return worker->imm_collide(a, b);
}




