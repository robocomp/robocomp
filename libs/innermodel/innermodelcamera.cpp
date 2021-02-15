/*
 * Copyright 2016 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "innermodel/innermodelcamera.h"
#include <innermodel/innermodel.h>

InnerModelCamera::InnerModelCamera(QString id_, float width_, float height_, float focal_, InnerModel *innermodel_, InnerModelNode *parent_) 
: InnerModelNode(id_, parent_), innermodel(innermodel_)
{
#if FCL_SUPPORT==1
	collisionObject = NULL;
#endif
	camera = Cam(focal_, focal_, width_/2., height_/2.);
	camera.setSize(width, height);
	width = width_;
	height = height_;
	focal = focal_;
}

void InnerModelCamera::print(bool verbose)
{
	if (verbose) camera.print(QString("Camera: ")+id);
}

void InnerModelCamera::update()
{
	if (fixed)
	{
	}
	updateChildren();
}

void InnerModelCamera::save(QTextStream &out, int tabs)
{
	for (int i=0; i<tabs; i++) out << "\t";
	out << "<camera id=\"" << id << "\" width=\"" << QString::number(width, 'g', 10) << "\" height=\"" << QString::number(height, 'g', 10) << "\" focal=\"" << QString::number(camera.getFocal(), 'g', 10) << "\" />\n";
}

InnerModelNode * InnerModelCamera::copyNode(QHash<QString, InnerModelNode *> &hash, InnerModelNode *parent)
{
	InnerModelCamera *ret = new InnerModelCamera(id, width, height, focal, innermodel, parent);
	ret->level = level;
	ret->fixed = fixed;
	ret->children.clear();
	ret->attributes.clear();
	hash[id] = ret;
	ret->innerModel = parent->innerModel;

	ret->innerModel = parent->innerModel;

	ret->camera = camera;

	for (QList<InnerModelNode*>::iterator i=children.begin(); i!=children.end(); i++)
	{
		ret->addChild((*i)->copyNode(hash, ret));
	}

	return ret;
}

QVec InnerModelCamera::project(QString reference, QVec origVec)
{
	origVec = innermodel->transform(id, origVec, reference);
	return project(origVec);
}

QVec InnerModelCamera::project(const QVec &origVec)
{
	QVec pc = camera.project(origVec);
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
QVec InnerModelCamera::backProject( const QString &cameraId, const QVec &	coord) //const
{
	return camera.getRayHomogeneous(coord);
}

void InnerModelCamera::imageCoordToAngles(const QString &cameraId, QVec coord, float &pan, float &tilt, const QString & anglesRefS)
{
	QVec ray = backProject(cameraId, coord);

	QVec finalRay = innermodel->getRotationMatrixTo(anglesRefS, cameraId)*ray;

	pan = atan2(finalRay(0), finalRay(2));
	tilt = atan2(finalRay(1), finalRay(2));

}

QVec InnerModelCamera::anglesToImageCoord(const QString &cameraId, float pan, float tilt, const QString & anglesRefS)
{
	QVec p(3), ray(3);

	p(0) = tan(pan);
	p(1) = tan(tilt);
	p(2) = 1;

	ray = innermodel->getRotationMatrixTo(cameraId, anglesRefS) * p;
	ray(0)=ray(0)/ray(2);
	ray(1)=ray(1)/ray(2);
	ray(2)=1;

	return project(cameraId, ray);
}

QVec InnerModelCamera::imageCoordPlusDepthTo(QString cameraId, QVec coord, float depth, QString reference)
{
	//We obtain a 3D line (a,b,1) in camera reference system that can be parametrized in depth to obtain a point at "depth" from the camera.
	QVec p = backProject( cameraId, coord ) * depth;
	//Now we transform it to requested node of the robot.
	if(p.size()>0)
		return innermodel->transform(reference, p, cameraId);
	return p;
}

QVec InnerModelCamera::projectFromCameraToPlane(const QString &to, const QVec &coord, const QString &cameraId, const QVec &vPlane, const float &dist)
{
	QMat mSystem(3,3);
	QVec tIndep(3);
	QVec pCam(3);
	QVec res(3);
	float dxz, dyz;

	pCam(0) = -coord(0) + getWidth()/2;
	pCam(1) = -(coord(1)-getHeight()/2);
	pCam(2) = getFocal();
	QVec pDest = innermodel->transform(to, pCam, cameraId);
	QVec pCent = innermodel->transform(to, QVec::vec3(0,0,0), cameraId);
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
// bool InnerModelCamera::check3DPointInsideFrustrum(QString cameraId, QVec coor)
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
QVec InnerModelCamera::horizonLine(QString planeId, QString cameraId, float heightOffset)
{
	
	// 	printf("-------------------------------------- cam:%s plane:%s\n", qPrintable(cameraId), qPrintable(planeId));
	// Get camera and plane pointers
	InnerModelPlane *plane = innermodel->getNode<InnerModelPlane>(planeId);
	InnerModelCamera *camera = innermodel->getNode<InnerModelCamera>(cameraId);
	// Transform rotate plane normal vector to camera reference system
	QMat rtm = innermodel->getRotationMatrixTo(cameraId, planeId);
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
	p1 = project(cameraId, p1);
	p2 = project(cameraId, p2);
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

QMat InnerModelCamera::getHomographyMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	

	QVec planeN = innermodel->getNode<InnerModelPlane>(plane)->normal;
	planeN = innermodel->getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = innermodel->transform(sourceCamera, innermodel->getNode<InnerModelPlane>(plane)->point, plane);
	QMat R  = innermodel->getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = innermodel->transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = innermodel->getNode<InnerModelCamera>(sourceCamera)->camera;
	QMat K2 = innermodel->getNode<InnerModelCamera>(virtualCamera)->camera;

	double d = -(planePoint*planeN);
	QMat H = K2 * ( R - ((t*n.transpose()) / d) ) * K1.invert();
	return H;
}

QMat InnerModelCamera::getAffineHomographyMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	

	QVec planeN = innermodel->getNode<InnerModelPlane>(plane)->normal;
	planeN = innermodel->getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = innermodel->transform(sourceCamera, innermodel->getNode<InnerModelPlane>(plane)->point, plane);

	QMat R  = innermodel->getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = innermodel->transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = innermodel->getNode<InnerModelCamera>(sourceCamera)->camera;

	double d = -(planePoint*planeN);
	QMat H = ( R - ((t*n.transpose()) / d) ) * K1.invert();
	for (int r=0;r<2;r++)
		for (int c=0;c<3;c++)
			H(r,c) = H(r,c) * 1000.;
	return H;
}

QMat InnerModelCamera::getPlaneProjectionMatrix(QString virtualCamera, QString plane, QString sourceCamera)
{
	
	QVec planeN = innermodel->getNode<InnerModelPlane>(plane)->normal;
	planeN = innermodel->getRotationMatrixTo(sourceCamera, plane)*planeN;
	QVec planePoint = innermodel->transform(sourceCamera, innermodel->getNode<InnerModelPlane>(plane)->point, plane);

	QMat R  = innermodel->getRotationMatrixTo(virtualCamera, sourceCamera);
	QMat t  = innermodel->transform(virtualCamera, QVec::vec3(0,0,0), sourceCamera);
	QMat n  = QMat(planeN);
	QMat K1 = innermodel->getNode<InnerModelCamera>(sourceCamera)->camera;

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

QVec InnerModelCamera::compute3DPointFromImageCoords(const QString &firstCamera, const QVec &left, const QString &secondCamera, const QVec &right, const QString &refSystem)
{
	
	QVec pI(3), pD(3), n(3), ray(3), T(3), TI(3), TD(3), pR(0), abc(3);
	QMat A(3,3);

	ray = backProject(firstCamera, left);
	pI = innermodel->getRotationMatrixTo(refSystem, firstCamera)*ray;
	pI(0)=pI(0)/pI(2);
	pI(1)=pI(1)/pI(2);
	pI(2)=1.;

	ray = backProject(secondCamera, right);
	pD = innermodel->getRotationMatrixTo(refSystem, secondCamera)*ray;
	pD(0)=pD(0)/pD(2);
	pD(1)=pD(1)/pD(2);
	pD(2)=1.;

	n = pI ^ pD;

	A(0,0)=pI(0);  A(0,1)=-pD(0);  A(0,2)=n(0);
	A(1,0)=pI(1);  A(1,1)=-pD(1);  A(1,2)=n(1);
	A(2,0)=pI(2);  A(2,1)=-pD(2);  A(2,2)=n(2);

	TI = innermodel->getTranslationVectorTo(refSystem, firstCamera).fromHomogeneousCoordinates();
	TD = innermodel->getTranslationVectorTo(refSystem, secondCamera).fromHomogeneousCoordinates();
	T = TD - TI ;

	abc = (A.invert())*T;

	pR = (pI*abc(0));
	pR = pR + TI;
	pR = (n*(abc(2)/2)) + pR;

	return pR;
}

QVec InnerModelCamera::compute3DPointFromImageAngles(const QString &firstCamera , const QVec & left, const QString & secondCamera , const QVec & right, const QString & refSystem)
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

	TI = innermodel->getTranslationVectorTo(refSystem, firstCamera).fromHomogeneousCoordinates();
	TD = innermodel->getTranslationVectorTo(refSystem, secondCamera).fromHomogeneousCoordinates();
	T = TD - TI;

	abc = (A.invert())*T;

	pR = (pI*abc(0));
	pR = pR + TI;
	pR = (n*(abc(2)/2)) + pR;

	return pR;
}

void InnerModelCamera::updateValues(float width_, float height_, float focal_)
{
	width = width_;
	height = height_;
	focal = focal_;
	camera = Cam(focal_, focal_, width_/2., height_/2.);
	camera.setSize(width, height);
}
