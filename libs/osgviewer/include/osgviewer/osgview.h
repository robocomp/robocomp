/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#ifndef OSGVIEW_H
#define OSGVIEW_H

/**
	@author Pablo Bustos and RoboLab's team
*/

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/GraphicsWindow>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <QtGui>
#include <QtOpenGL/QGLWidget>
#include <osgviewer/adapterwidget.h>

#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Shape>
#include <osg/Point>
#include <osg/Quat>
#include <osg/Image>
#include <osg/ShapeDrawable>
#include <osg/Camera>
#include <osg/Geometry>
#include <osg/LineSegment>
#include <osg/TexMat>
#include <osgUtil/Tessellator>
#include <osgUtil/IntersectVisitor>
//#include <osgviewer/findnamednode.h>
#include <osgviewer/viewerqt.h>
#include <osg/io_utils>

#include <qmat/QMatAll>

using Qt::WindowFlags;
using namespace RMat;

class OsgView : public QGLWidget, public osgViewer::Viewer
{
Q_OBJECT
public:

	struct laserMeasure{
		float dist;
		float angle;
	};
	osg::Geode* TextGeode;
	osgText::Text* textOne;

	void setClearColor(QVec color) { getCamera()->setClearColor(osg::Vec4(color(0), color(1), color(2), (color.size()>3?color(3):1))); }
// 	QVector<laserMeasure> laserData;

	OsgView(QWidget *parent=NULL);
	//OsgView(const OsgView&) {}
	OsgView(QWidget *parent, bool hud, const QGLWidget * shareWidget = 0, WindowFlags f = 0);
	void init(QWidget *parent, bool hud, const QGLWidget* shareWidget, WindowFlags f);
	~OsgView();
	virtual void paintGL();
	osgViewer::GraphicsWindow* getGraphicsWindow() { return _gw.get(); }
	const osgViewer::GraphicsWindow* getGraphicsWindow() const { return _gw.get(); }
	void addCamaraManipulator(int n, std::string name, osg::ref_ptr<osgGA::TrackballManipulator> man);
	void setScene( osg::ref_ptr<osg::Group> root ){ setSceneData( root.get() );};
	void setTextHUD (std::string text, osg::Vec3 position=osg::Vec3(320.,10.,0.), osg::Vec4 color = osg::Vec4(1, 1, 1, 1))
	{
		textOne->setCharacterSize(80);
		textOne->setText(text);
		textOne->setPosition(position);
		textOne->setColor(color);
	}
	void setHomePosition (osg::Vec3 eye, osg::Vec3 center, osg::Vec3 up, bool autoComputeHomePosition);
	void init();
	void initHUD ();
	void addXYZAxisOnNode( osg::Group *node, float length, float radius , const osg::Vec4 &color = osg::Vec4(1.,0.,0.,0.));
	void addCosa(osg::Node *node);

	osg::ShapeDrawable *addRectangle(float px, float py, float pz, float w, float h, osg::PositionAttitudeTransform **pat);

	osg::ShapeDrawable *addBox(float px, float py, float pz, float sx, float sy, float sz, osg::PositionAttitudeTransform **pat, osg::Group *parentNode=NULL);
	osg::ShapeDrawable *addBox(float px, float py, float pz, float sx, float sy, float sz, osg::PositionAttitudeTransform **pat);

	osg::ref_ptr<osg::PositionAttitudeTransform> addThing(const QString & name,const RMat::QMat &center , float orientation );
	osg::ref_ptr<osg::Node> addBall(const QVec &center, float radius, const QVec &color = QVec::vec4(0.f,1.f,0.f,0.f));
	osg::ref_ptr<osg::Node> addBall(const osg::Vec3 & center, float radius, const osg::Vec4 &color = osg::Vec4(0.f,1.f,0.f,0.f));

	void removeBall( osg::Node * b );
	osg::ref_ptr<osg::Node> addBox(const QVec &center, float side, const osg::Vec4 &color = osg::Vec4(0.f,1.f,0.f,0.f));
	void addPlane(const QMat &center, float w, float h);
	void removeBox( osg::Node * b );
	osg::Node * addLine(const QVec& p1, const QVec& p2, float radius, bool tip = false);
	osg::Node * addBasicLine(const QVec & p1, const QVec & p2, float radius, bool tip=false, const osg::Vec4 &color = osg::Vec4(1.,0.,0.,0.));
	osg::Node * addArrow(const QMat &p1, const QMat &p2, float radius);
	osg::ref_ptr<osg::Node> addPolyLine(const QVector< QVec > & pl, float radius);
	osg::ref_ptr<osg::Node> addPolygon(const osg::Vec3Array &v,const osg::Vec4 &color = osg::Vec4(1.0f, 0.0, 1.0f, 0.20f));
	void removePolyLine( osg::Node * pl );
	osg::Shape *addBasicLineShape(const QVec &p1, const QVec &p2, float radius);
	osg::Node * hexno = NULL;
	osg::Vec3 kk;
	int flag1 = 0, flag2=0;

	osg::Group * getRootGroup()	{ return root; }

	void autoResize() {
		if (parentWidget()) {
			setFixedSize(parentWidget()->width(), parentWidget()->height());
		}
		getCamera()->setProjectionMatrixAsPerspective(70.0f, static_cast<double>(width())/static_cast<double>(height()), 0.00000001, 1000000.0);
	}

  void autoResize(float HFOV) {
    if (parentWidget()) {
      setFixedSize(parentWidget()->width(), parentWidget()->height());
    }
    getCamera()->setProjectionMatrixAsPerspective(HFOV, static_cast<double>(width())/static_cast<double>(height()), 0.00000001, 1000000.0);
  }

	void handle( const QPoint & p);
	//Picking objects
	void pickObject( const QPoint & p);
	void setImageHUD(osg::ref_ptr<osg::Image> i);
	osg::ref_ptr<osg::Image> getImageHUD ();
	osg::Camera* createHUD();


protected:
	QTimer timer;
	virtual void resizeGL( int width, int height );
	virtual void keyPressEvent( QKeyEvent* event );
	virtual void keyReleaseEvent( QKeyEvent* event );
	virtual void mousePressEvent( QMouseEvent* event );
	virtual void mouseReleaseEvent( QMouseEvent* event );
	virtual void mouseMoveEvent( QMouseEvent* event );
	osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw;
	osg::Quat quaternionFromRotationMatrix(const QMat &rotM) const;
	osg::Quat quaternionFromInitFinalVector(const QVec &v1, const QVec &v2) const;
private:
	osg::ref_ptr<osg::Group> root;
	osg::ref_ptr<osg::Image> osgImage;
	osg::ref_ptr<osg::Texture2D> HUDTexture;

public:
	QMutex mutex;

	inline void printFPS();
	static osg::Vec3 qmatToVec3(const RMat::QMat & m);
	static osg::Vec3 qvecToVec3(const RMat::QVec & m) { return osg::Vec3(m(0),m(1),m(2)); }
	static osg::Vec4 qvecToVec4(const RMat::QVec & m);
	static RMat::QMat vec3ToQMat(const osg::Vec3f & m);
	static RMat::QVec vec3ToQVec(const osg::Vec3f & m);
	static RMat::QVec vec4ToQVec(const osg::Vec4f & m);

signals:
	void newWorld3DCoor( const QVec & vec);
	void newLeftCoor(QPointF p);
	void newRightCoor(QPointF p);
	void endCoor(QPointF p);
	void keyPress(QString t);
	void keyRelease(QString t);

};

#endif
