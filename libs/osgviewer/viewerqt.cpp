#include <osgviewer/viewerqt.h>

ViewerQT::ViewerQT(QWidget * parent, const char * name, const QGLWidget * shareWidget, WindowFlags f) : AdapterWidget(parent, name, shareWidget, f), osgViewer::Viewer()
{
	getCamera()->setViewport(new osg::Viewport(0,0,width(),height()));
    getCamera()->setGraphicsContext(getGraphicsWindow());
    setThreadingModel(osgViewer::Viewer::SingleThreaded);
    //setThreadingModel(osgViewer::Viewer::DrawThreadPerContext );


	ksManipulator = new osgGA::KeySwitchMatrixManipulator;
//	osg::ref_ptr<osg::Material> mat = new osg::Material;
//	mat->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );
//	globalStateset->setAttribute( mat.get() );

    connect(&_timer, SIGNAL(timeout()), this, SLOT(updateGL()));
	// connect(&_timer, SIGNAL(timeout()), this, SLOT(paintGL()));
   //  _timer.start(200);
}

ViewerQT::~ViewerQT()
{
}

void ViewerQT::paintGL()
{
	//printFPS( );
    frame();
	//updateGL();
}

void ViewerQT::addCameraManipulator(int n, std::string name, osg::ref_ptr< osgGA::TrackballManipulator > man)
{
	ksManipulator->addMatrixManipulator( n, name, man.get() );
}

void ViewerQT::keyPressEvent(QKeyEvent * event)
{
		qDebug() <<"hola";
		ksManipulator->selectMatrixManipulator( event->key() );
		setCameraManipulator( ksManipulator->getCurrentMatrixManipulator() );


}

/*uchar * ViewerQT::getGrayBuffer()
{
	uchar buf[320*240];
	glReadBuffer(GL_FRONT);
	glReadPixels(0,0,320,240,GL_LUMINANCE,GL_UNSIGNED_BYTE,buf);
	return buf;
}*/

void ViewerQT::printFPS( )
{

	static int fps=0;
	static QTime ti;
	if ((++fps % 50) == 0)
	{
		std::cout << "fps " << 50000 / ti.restart() << std::endl;
	}
}
