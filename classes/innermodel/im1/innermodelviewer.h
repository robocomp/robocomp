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

#ifndef INNERMODELVIEWER_H
#define INNERMODELVIEWER_H

// OSG includes
#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Image>
#include <osg/LineSegment>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/Quat>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/TexMat>
#include <osgDB/ReadFile>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgUtil/IntersectVisitor>
#include <osgViewer/CompositeViewer>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

// Qt includes
#include <QHash>
#include <QtGui>
#include <QtOpenGL/QGLWidget>

// Robocomp includes
#include <osgviewer/adapterwidget.h>
#include <osgviewer/findnamednode.h>
#include <osgviewer/getworldcoorofnode.h>
#include <osgviewer/viewerqt.h>
#include <QMat/QMatAll>
#include "innermodel.h"



osg::Vec3 QVecToOSGVec(const QVec &vec);
osg::Vec4 htmlStringToOsgVec4(QString color);
QString osgVec4ToHtmlString(osg::Vec4 color);
osg::Matrix QMatToOSGMat4(const RTMat &nodeB);



struct IMVCamera
{
	osg::Image *rgb;
	osg::Image *d;
	InnerModelRGBD *RGBDNode;
	osgViewer::Viewer *viewerCamera;
	osgGA::TrackballManipulator *manipulator;
	QString id;
};



struct IMVLaser
{
	InnerModelLaser *laserNode;
	osg::Switch *osgNode;
	QString id;
};



struct IMVMesh
{
	osg::Node * osgmeshes;
	osg::MatrixTransform * osgmeshPaths;
	osg::MatrixTransform * meshMts;
};



class IMVPlane : public osg::Geode
{
	friend class InnerModelViewer;
public:
	IMVPlane(InnerModelPlane *plane, std::string imagenEntrada, osg::Vec4 valoresMaterial, float transparencia);
	void updateBuffer(uint8_t *data_, int32_t width_, int32_t height_);
	void performUpdate();
	
	// protected:
	uint8_t *data;
	bool dirty;
	int32_t width, height;
	osg::Texture2D* texture;
	osg::ShapeDrawable *planeDrawable;
	osg::Image *image;
};



class IMVPointCloud : public osg::Geode
{
public:
	IMVPointCloud(std::string id_);
	void update();
	float getPointSize();
	void setPointSize(float p);
	
	std::string id;
	osg::Vec3Array *points;
	osg::Vec4Array *colors;
	
protected:
	osg::Vec3Array *cloudVertices;
	osg::Vec4Array *colorsArray;
	osg::Geometry *cloudGeometry;
	osg::TemplateIndexArray <unsigned int, osg::Array::UIntArrayType,4,4> *colorIndexArray;
	osg::DrawArrays *arrays;
	float pointSize;
};



class InnerModelViewer : public osg::Switch
{
public:
	enum CameraView { BACK_POV, FRONT_POV, LEFT_POV, RIGHT_POV, TOP_POV };
	
	InnerModelViewer(InnerModel *im, QString root="root", osg::Group *parent=NULL);
	~InnerModelViewer();
	
	///Return geode id if no geode, return null.
	osg::Geode* getGeode(QString id);
	void update();
	void reloadMesh(QString id);
	
	// 	void recursiveConstructor(InnerModelNode *node, osg::Group *parent, QHash<QString, osg::MatrixTransform *> &mtsHash, QHash<QString, osg::Node *> &osgmeshesHash, QHash<QString, osg::MatrixTransform *> &osgmeshPatsHash);
	void recursiveConstructor(InnerModelNode* node, osg::Group* parent, QHash< QString, osg::MatrixTransform* >& mtsHash, QHash< QString, IMVMesh >& meshHash);
	void setMainCamera(osgGA::TrackballManipulator *manipulator, CameraView pov) const;
	
protected:
	void setOSGMatrixTransformForPlane(osg::MatrixTransform *mt, InnerModelPlane *plane);
	InnerModel *innerModel;
	
public:
	//QHash<QString, osg::Node *> osgmeshes;
	QHash<QString, osg::PolygonMode *> osgmeshmodes;
	// 	QHash<QString, osg::MatrixTransform *> osgmeshPats;
	QHash<QString, osg::MatrixTransform *> mts;
	QHash<QString, osg::MatrixTransform *> planeMts;
	// 	QHash<QString, osg::MatrixTransform *> meshMts;
	QHash<QString, IMVMesh> meshHash;
	QHash<QString, IMVPointCloud *> pointCloudsHash;
	QHash<QString, IMVPlane *> planesHash;
	QHash<QString, IMVCamera> cameras;
	QHash<QString, IMVLaser> lasers;
};

#endif
