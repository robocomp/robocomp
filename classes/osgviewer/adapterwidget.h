#ifndef ADAPTERWIDGET_H
#define ADAPTERWIDGET_H

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>

#include <QString>
#include <QTimer>
#include <QKeyEvent>
#include <QtOpenGL/QGLWidget>
    
using Qt::WindowFlags;

#include <iostream>

/**
	@author authorname <authormail>
*/
class AdapterWidget : public QGLWidget
{
Q_OBJECT
public:
    AdapterWidget( QWidget * parent = 0, const char * name = 0, const QGLWidget * shareWidget = 0, WindowFlags f = 0 );
    virtual ~AdapterWidget(){};
    osgViewer::GraphicsWindow* getGraphicsWindow() { return _gw.get(); }
    const osgViewer::GraphicsWindow* getGraphicsWindow() const { return _gw.get(); }
 protected:
    void init();
    virtual void resizeGL( int width, int height );
    virtual void keyPressEvent( QKeyEvent* event );
    virtual void keyReleaseEvent( QKeyEvent* event );
    virtual void mousePressEvent( QMouseEvent* event );
    virtual void mouseReleaseEvent( QMouseEvent* event );
    virtual void mouseMoveEvent( QMouseEvent* event );
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw;
};

#endif
