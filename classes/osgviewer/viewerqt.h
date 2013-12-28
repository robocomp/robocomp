#ifndef VIEWERQT_H
#define VIEWERQT_H

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <QtGui>
#include <QtOpenGL/QGLWidget>
#include <osgviewer/adapterwidget.h>

/**
	@author authorname <authormail>
*/
class ViewerQT : public AdapterWidget, public osgViewer::Viewer
{
Q_OBJECT
public:
    ViewerQT(QWidget * parent = 0, const char * name = 0, const QGLWidget * shareWidget = 0, WindowFlags f = 0);
  	~ViewerQT();
    virtual void paintGL();
	void addCameraManipulator(int n, std::string name, osg::ref_ptr<osgGA::TrackballManipulator> man);

    
protected:
        QTimer _timer;

private:
	osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> ksManipulator;
	void printFPS();

public slots:
	void keyPressEvent ( QKeyEvent * event ) ;
};

#endif
