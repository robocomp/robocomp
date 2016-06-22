/*
 *    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License aas published by
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
#include <cmath>

#include "rcdraw.h"

RCDraw::RCDraw( int _width, int _height, uchar *img, QWidget *parent) : QGLWidget(parent), width(_width), height(_height)
{
	resize(width, height);
	win.setRect( 0, 0, width, height);

	qimg = NULL;
	if (img != NULL)
		qimg = new QImage(img, width, height, QImage::Format_Indexed8);

	init();
}

RCDraw::RCDraw( int _width, int _height, uchar *img, QImage::Format format , QWidget *parent) : QGLWidget(parent), width(_width), height(_height)
{
	resize ( width, height );
	win.setRect ( 0, 0, width, height );

	qimg = NULL;
	if ( img != NULL )
		qimg = new QImage ( img, width, height, format );
	init();
}

RCDraw::RCDraw(int _width, int _height, QImage *img, QWidget *parent) : QGLWidget(parent), width (_width), height (_height)
{
	resize (width,height );
	win.setRect ( 0, 0, width, height );

	qimg = NULL;
	if (img)
		qimg = img;

	init();
}

RCDraw::RCDraw(QImage * img, QWidget *parent) : QGLWidget ( parent )
{
	if (parent)
	{
		width = parent->width();
		height = parent->height();
	}
	win.setRect ( 0, 0, width, height );

	qimg = NULL;
	if (img)
		qimg = img;

	init();
}



RCDraw::RCDraw(int _width, int _height, QWidget * parent): QGLWidget (parent), width (_width), height (_height)
{
	resize ( _width, _height );
	win.setRect ( 0,0,width,height );

	linGrad.setStart ( width, height );
	linGrad.setFinalStop ( width, height-150 );
	linGrad.setSpread ( QGradient::PadSpread );
	qimg = NULL;
	init();
}

RCDraw::RCDraw(QWidget * parent): QGLWidget (parent)
{
	if (parent)
	{
		setParent ( parent );
		resize ( parent->width(), parent->height() );
		win.setRect ( 0,0,parent->width(),parent->height() );
	}
	qimg=NULL;
	init();
}


RCDraw::RCDraw(const QRect &win_, QWidget *parent) : QGLWidget(parent)
{
	if (parent)
	{
		setParent ( parent );
		width = parent->width();
		height = parent->height();
		resize ( parent->width(), parent->height() );
	}
	setWindow ( win_ );
	qimg=NULL;
	init();
}

RCDraw::~RCDraw()
{
}

bool RCDraw::autoResize(bool ignoreAspectRatio)
{ 
	if (parent())
	{
		if (parentWidget()->width() != (int)getWidth() or parentWidget()->height() != (int)getHeight())
		{
			setFixedSize(parentWidget()->width(), parentWidget()->height()); 
			width = parentWidget()->width(); 
			height = parentWidget()->height();
			if (ignoreAspectRatio)
			{
				const int32_t ww = getWidth();
				const int32_t hh = getHeight();
				setWindow(QRect(0,0, ww*zoomMul, hh*zoomMul));
			}
			if (qimg)
			{
				qimg->scaled(width,height);
			}
			return true;
		}
	}
	return false;
}

void RCDraw::init( )
{
	zoomMul = 0.5;
	invertedVerticalAxis=false;
	visibleCenter = QVec(2,0);
	DRAW_AXIS = false;
	DRAW_PERIMETER = false;
	imageScale = 1.;
	if (qimg!=NULL)
	{
		imageScale = width/qimg->width();
	}
	else
	{
		qimg = new QImage ( width, height, QImage::Format_Indexed8 );
		qimg->fill ( 240 );
	}
	//Gray color table
	ctable.resize ( 256 );
	for ( int i=0; i < 256; i++ )
	{
		ctable[i] = qRgb ( i,i,i );
	}
	qimg->setColorTable ( ctable );
	translating = false;
	effWin = win;
	QGLFormat f = format();
	if (f.sampleBuffers())
	{
		f.setSampleBuffers( true );
		setFormat( f );
		std::cout << "Sample Buffers On in QGLWidget" << std::endl;
	}
	else
	{
		std::cout << "Sample Buffers Off in QGLWidget" << std::endl;
	}
	show();
}


void RCDraw::setImage(QImage* img)
{
	if (qimg != NULL) 
		delete qimg;
	
	qimg = img;
}

void RCDraw::paintEvent ( QPaintEvent * )
{
	QString s;
	QPainter painter ( this );
	painter.setRenderHint(QPainter::HighQualityAntialiasing);

	if ( qimg != NULL )
	{
		painter.drawImage ( QRectF(0., 0., imageScale*width, imageScale*height), *qimg, QRectF(0, 0, width, height) );
	}

	painter.setWindow (effWin.toRect() );

	if ( DRAW_PERIMETER )
	{
		painter.setPen ( Qt::blue );
		painter.drawRect ( 0,0,width-1,height-1 );
	}
	if ( DRAW_AXIS )
	{
		drawAxis(Qt::blue, 2);
	}

	//Draw lines
	while ( !lineQueue.isEmpty() )
	{
		TLine l = lineQueue.dequeue();
		if (invertedVerticalAxis) l.line = QLineF(l.line.x1()-visibleCenter(0), -l.line.y1()+visibleCenter(1), l.line.x2()-visibleCenter(0), -l.line.y2()+visibleCenter(1));
		else l.line.translate(-visibleCenter(0), -visibleCenter(1));
		painter.setPen ( QPen ( QBrush ( l.color ),l.width ) );
		painter.drawLine ( l.line );
	}

	//Draw gradient
	while ( !gradQueue.isEmpty() )
	{
		TGrad g = gradQueue.dequeue();
		if (invertedVerticalAxis) g.line = QLine(g.line.x1()-visibleCenter(0), -g.line.y1()+visibleCenter(1), g.line.x2()-visibleCenter(0), -g.line.y2()+visibleCenter(1));
		else g.line.translate(-visibleCenter(0), -visibleCenter(1));
		linGrad.setColorAt ( 0, g.color );
		linGrad.setColorAt ( 1, g.color1 );
		painter.setBrush ( linGrad );
		painter.setPen ( QPen ( linGrad, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin ) );
		painter.drawLine ( g.line );
	}

	//Draw ellipses
	while ( !ellipseQueue.isEmpty() )
	{
		TEllipse e = ellipseQueue.dequeue();
		if (invertedVerticalAxis) e.center.setY(-(e.center.y()-visibleCenter(1)));
		else e.center = QPointF(e.center.x()-visibleCenter(0), e.center.y()-visibleCenter(1));
		if ( e.fill == true )
			painter.setBrush ( e.color );
		else
			painter.setBrush ( Qt::transparent );
		painter.setPen ( e.color );
		if (fabs(e.ang) > 0.1)
		{
			painter.setPen ( e.color );
			painter.translate( e.center );
			painter.rotate( e.ang );
			painter.drawEllipse ( QPointF(0,0), e.rx, e.ry );
			painter.rotate( -e.ang );
			painter.translate( -e.center );
		}
		else
			painter.drawEllipse( e.center, e.rx, e.ry);
	}

	//Draw squares
	{
		QPen pen = painter.pen();
		int penwidth = pen.width();
		while ( !squareQueue.isEmpty() )
		{
			TRect r = squareQueue.dequeue();
			if (invertedVerticalAxis) r.rect = QRect(r.rect.x()-visibleCenter(0), -r.rect.y()+visibleCenter(1)-r.rect.height(), r.rect.width(), r.rect.height());
			else r.rect.translate(-visibleCenter(0),-visibleCenter(1));
			if ( r.fill == true )
				painter.setBrush ( r.color );
			else
				painter.setBrush ( Qt::transparent );
			pen.setColor(r.color);
			pen.setWidth(r.width);
			painter.setPen(pen);
			if (fabs(r.ang) > 0.01 )
			{
				QPointF center = r.rect.center();
				painter.translate( center );
				painter.rotate( r.ang );
				painter.drawRect ( QRectF( r.rect.topLeft() - center, r.rect.size() ));
				painter.rotate( -r.ang );
				painter.translate( -center );
			}
			else
				painter.drawRect( r.rect );
		}
		pen.setWidth(penwidth);
		painter.setPen(pen);
	}


	while ( !lineOnTopQueue.isEmpty() )
	{
		TLine l = lineOnTopQueue.dequeue();
		l.line.translate(-visibleCenter(0), -visibleCenter(1));
		painter.setPen ( QPen ( QBrush ( l.color ),l.width ) );
		painter.drawLine ( l.line );
	}


	//Draw text
	while ( !textQueue.isEmpty() )
	{
		TText t = textQueue.dequeue();
		painter.setWindow ( effWin.normalized().toRect() );
		QFont ant = painter.font();
		QFont temp ( "Helvetica", t.size );
		painter.setFont ( temp );

		QRectF rect = painter.boundingRect(QRectF(t.pos.x(), t.pos.y(), 1, 1), Qt::AlignLeft, t.text);
		if (not t.centered)
		{
			painter.setPen(t.bgc);
			painter.setBrush(t.bgc);
			painter.drawRect(rect);
			painter.setBrush ( Qt::transparent );
			painter.setPen ( t.color );
			painter.drawText( QRectF(t.pos.x(), t.pos.y(), 0.82*t.text.size()*t.size, 1.2*t.size), Qt::AlignCenter, t.text);
		}
		else
		{
			rect.translate(-rect.width()/2., -rect.height()/2.);
			painter.setPen(t.bgc);
			painter.setBrush(t.bgc);
			painter.drawRect(rect);
			painter.setBrush ( Qt::transparent );
			painter.setPen ( t.color );
			painter.drawText(rect, Qt::AlignLeft, t.text);
		}

		painter.setFont ( ant );
		painter.setWindow ( effWin.toRect() );
	}


}


void RCDraw::drawSquare ( const QRect &rect, const QColor & col, bool fill, int id, float rot, float width)
{
	TRect r;
	r.rect = rect;
	r.color= col;
	r.id = id;
	r.ang = rot;
	r.fill = fill;
	r.width = width;
	squareQueue.enqueue ( r );
}


void RCDraw::drawSquare ( const QPoint & center, int sideX, int sideY, const QColor & col, bool fill, int id, float rads, float width)
{
	TRect r;
	r.rect = QRect ( center.x()-sideX/2, (center.y()-sideY/2), sideX, sideY );
	r.rect.moveCenter(center);
	r.color= col;
	r.id = id;
	r.fill = fill;
	r.ang = rads*180./M_PI;
	r.width = width;
	squareQueue.enqueue ( r );
}

void RCDraw::drawSquare ( const QPointF & center, float sideX, float sideY, const QColor & col, bool fill, int id, float rads, float width)
{
	TRect r;
	r.rect = QRectF ( center.x()-sideX/2, center.y()-sideY/2, sideX, sideY );
	r.color= col;
	r.id = id;
	r.fill = fill;
	r.ang = rads*180./M_PI;
	r.width = width;
	squareQueue.enqueue ( r );
}


void RCDraw::drawLine ( const QLine & line, const QColor & c, float width )
{
	TLine l;
	l.line=line;
	l.color=c;
	l.width = width;
	lineQueue.enqueue ( l );
}

void RCDraw::drawLine ( const QLineF & line, const QColor & c, float width )
{
	TLine l;
	l.line=line;
	l.color=c;
	l.width = width;
	lineQueue.enqueue ( l );
}

// void RCDraw::drawLineF ( const QLineF & line, const QColor & c, float width )
// {
// 	TLine l;
// 	l.line=line;
// 	l.color=c;
// 	l.width = width;
// 	qDebug() << "Deprecated. Use overloaded version (RCDraw::drawline())";
// 	lineQueue.enqueue ( l );
// }


void RCDraw::drawLineOnTop ( const QLine & line, const QColor & c, float width )
{
	TLine l;
	l.line=line;
	l.color=c;
	l.width = width;
	lineOnTopQueue.enqueue ( l );
}

void RCDraw::drawLineOnTop ( const QLineF & line, const QColor & c, float width )
{
	TLine l;
	l.line=line;
	l.color=c;
	l.width = width;
	lineOnTopQueue.enqueue ( l );
}

void RCDraw::drawLineFOnTop ( const QLineF & line, const QColor & c, float width )
{
	TLine l;
	l.line=line;
	l.color=c;
	l.width = width;
	lineOnTopQueue.enqueue ( l );
	qDebug() << "Deprecated. Use overloaded version (RCDraw::drawline())";
}

void RCDraw::drawPolyLine ( const QVector< QPoint > & pline, const QColor & c, int width )
{
	TLine l;
	if ( pline.size() > 1 )
	{
		for ( int i=1; i< pline.size(); i++ )
		{
			l.line.setPoints ( pline[i-1],pline[i] );
			l.color=c;
			l.width = width;
			lineQueue.enqueue ( l );
		}
	}
}
void RCDraw::drawPolyLine(const QVector< QPointF >& pline, const QColor& c, int width)
{
	TLine l;
	if ( pline.size() > 1 )
	{
		for ( int i=1; i< pline.size(); i++ )
		{
			l.line.setPoints ( pline[i-1],pline[i] );
			l.color=c;
			l.width = width;
			lineQueue.enqueue ( l );
		}
	}

}


void RCDraw::drawPolyLine ( const QVector< int > & xs, const QVector< int > & ys, const QColor & c, int width )
{
	TLine l;
	QPoint pant;
	if ( xs.size() > 1 )
	{
		pant.setX ( xs[0] );
		pant.setY ( ys[0] );
		for ( int i=1; i< xs.size(); i++ )
		{
			l.line.setPoints ( pant,QPoint ( xs[i],ys[i] ) );
			l.color=c;
			l.width = width;
			lineQueue.enqueue ( l );
			pant.setX ( xs[i] );
			pant.setY ( ys[i] );
		}
	}
}

void RCDraw::drawGrad ( const QLine & line, const QColor & c, const QColor & c1, float width )
{
	TGrad g;
	g.line=line;
	g.color=c;
	g.color1=c1;
	g.width = width;
	gradQueue.enqueue ( g );
}

void RCDraw::drawEllipse ( const QRectF & rect, const QColor &col, bool fill, int id , float rads)
{
	TEllipse e;
	e.rect = rect;
	e.color= col;
	e.id = id;
	e.fill = fill;
	e.ang = rads;
	ellipseQueue.enqueue ( e );
}

void RCDraw::drawEllipse ( const QRect & rect, const QColor &col, bool fill, int id , float rads)
{
	drawEllipse(QRectF(rect.x(), rect.y(), rect.width(), rect.height()), col, fill, id, rads);
}

void RCDraw::drawEllipse ( const QPoint & centro, int radiusX,  int radiusY, const QColor & col, bool fill, int id ,  float rads)
{
	TEllipse e;
	e.center = centro;
	e.rx = radiusX;
	e.ry = radiusY;
	e.rect = QRect(centro.x()-radiusX, centro.y()-radiusY, radiusX*2, radiusY*2);
	e.color= col;
	e.id = id;
	e.fill = fill;
	e.ang = rads*180./M_PI;
	ellipseQueue.enqueue ( e );

}

void RCDraw::drawEllipse ( const QPointF & centro, float radiusX,  float radiusY, const QColor & col, bool fill, int id ,  float rads)
{
	TEllipse e;
	e.center = centro;
	e.rx = radiusX;
	e.ry = radiusY;
	e.color= col;
	e.id = id;
	e.fill = fill;
	e.ang = rads*180./M_PI;
	ellipseQueue.enqueue ( e );
}

void RCDraw::draw2DRoiOnFloor(const QPoint & center, const QMat & cov, const QColor & col, bool fill, int id)
{
	//extract submatrix X;Z for 2D floor projection
	QMat cov2D(2,2);
 	cov2D = cov;

	QVec vals(2);
	QMat vecs = cov2D.eigenValsVectors ( vals );
	float sigma1 = vals(0);
	float sigma2 = vals(1);
	float phi;
	if (sigma1 > sigma2)
		phi = atan2(vecs(1,0),vecs(0,0));
	else
		phi = atan2(vecs(1,1),vecs(0,1));

	float ang = phi*180/M_PI + (M_PI/2);

	if ( std::isnan(sigma1)==false and std::isinf(sigma1)==false and std::isnan(sigma2)==false and std::isinf(sigma2)==false)
	{
		TEllipse e;
		e.center = center;
	 	e.rx = sigma1;
		e.ry = sigma2;
		e.color = col;
		e.id = id;
		e.fill = fill;
		e.ang = ang;

		ellipseQueue.enqueue ( e );
	}
}


void RCDraw::draw3DRoiOnFloor(const QVec & center, const QMat & cov, const QColor & col, bool fill, int id)
{
	//extract submatrix X;Z for 2D floor porjection
	QMat cov2D(2,2);
	/*cov2D(0,0) = cov(0,0);
	cov2D(0,1) = cov(0,2);
	cov2D(1,0) = cov(2,0);
	cov2D(1,1) = cov(2,2);
	*///cov2D.print("cov2D");
	cov2D = cov;

	QVec vals(2);
	QMat vecs = cov2D.eigenValsVectors ( vals );
	float sigma1 = vals(0);
	float sigma2 = vals(1);
	float phi;
	if (sigma1 > sigma2)
		phi = atan2(vecs(1,0),vecs(0,0));
	else
		phi = atan2(vecs(1,1),vecs(0,1));

	float ang = phi*180/M_PI + (M_PI/2);
	qDebug() << "ang " << phi;
/*	if (phi<0)
		ang = phi*180/M_PI + 360;
	else
		ang = phi*180/M_PI;*/

	if ( std::isnan(sigma1)==false and std::isinf(sigma1)==false and std::isnan(sigma2)==false and std::isinf(sigma2)==false)
	{
		TEllipse e;
		e.center.setX( center.x());
		e.center.setY( center.z());
	 	e.rx = sigma1;
		e.ry = sigma2;
		e.color = col;
		e.id = id;
		e.fill = fill;
		e.ang = ang;
		ellipseQueue.enqueue ( e );
	}
}

void RCDraw::drawText ( const QPointF & pos, const QString & text, int size, const QColor & color, bool centered, const QColor bgc)
{
	TText t;
	t.pos = pos;
	t.text = text;
	t.size = size;
	t.color = color;
	t.centered = centered;
	t.bgc = bgc;
	textQueue.enqueue ( t );
}



///Mouse events

void RCDraw::mousePressEvent ( QMouseEvent *e )
{
	if (e->button() == Qt::MidButton)
	{
		translating = true;
		backPos = e->pos();
		return;
	}
	else if ( (e->button() == Qt::RightButton) or (e->button() == Qt::LeftButton) )
	{
		inicio.setX ( e->x() );
		inicio.setY ( e->y() );

		emit iniMouseCoor( QPoint(e->x(), e->y()) );
		double xratio = ((double)effWin.width()) /((double)((QWidget*)this)->width());
		double zratio = -((double)effWin.height())/((double)((QWidget*)this)->height());
		float fx = ((float)effWin.left())   + xratio*e->x() + visibleCenter.x();
		float fz = ((float)effWin.bottom()) + zratio*e->y() + visibleCenter.y();
		emit newCoor( QPointF(fx, fz) );
		emit press(QPointF(fx, fz));

		if (e->button() == Qt::LeftButton)
			emit newLeftCoor( QPointF(fx, fz) );
		else
			emit newRightCoor( QPointF(fx, fz) );
		return;
	}
}

void RCDraw::mouseMoveEvent(QMouseEvent *e)
{
	if (translating)
	{
		const int mul = effWin.width()/getWidth();
		const int ix = mul*(backPos.x()-e->x())*((effWin.width()<0)?-1:1);
		const int iy = mul*(backPos.y()-e->y())*((effWin.height()<0)?-1:1);
// 		printf("T(%d,%d)\n", ix, iy);
		effWin.translate(ix, iy);
		backPos = e->pos();
	}
}

void RCDraw::mouseReleaseEvent ( QMouseEvent *e )
{
	emit endMouseCoor( QPoint(e->x(), e->y()) );
	emit endCoor(e->posF());
	inicio.setX ( e->x() );
	inicio.setY ( e->y() );

	double xratio = ((double)effWin.width()) /((double)((QWidget*)this)->width());
	double zratio = -((double)effWin.height())/((double)((QWidget*)this)->height());
	float fx = ((float)effWin.left())   + xratio*e->x() + visibleCenter.x();
	float fz = ((float)effWin.bottom()) + zratio*e->y() + visibleCenter.y();
	emit newCoor( QPointF(fx, fz) );
	emit release(QPointF(fx, fz));

	if (e->button() == Qt::RightButton)
	{
		translating = false;
// 		printf("Translating %d\n", translating);
		return;
	}
	
}

void RCDraw::drawPerimeter ( const QColor &c, int width, int margin )
{
	QRect perim ( effWin.x(), effWin.y(), effWin.width()-margin, effWin.height() +margin );
	drawSquare(effWin.toRect(), c);
	width=width;
}
void RCDraw::drawPerimeter ( const QColor &c, int width )
{
	/*QRect perim(effWin.x(),effWin.y(),effWin.width()-10,effWin.height()+10);*/
// 	QRect perim (effWin.x() +1, effWin.y()-1, effWin.width()-2, effWin.height() +1 );
// 	drawSquare ( perim , c );
// 	width=width;

	drawSquare(effWin.center(), effWin.width(), effWin.height(), Qt::darkCyan, false, width);
}

/**
 * @brief ... Draw vertical and horizontal axis on the frame. Marks are placed at step intervals in world reference frame
 *
 * @param c ... line color
 * @param w ... line width
 * @param step ...separation among axis marks
 * @return void
 **/
void RCDraw::drawAxis ( const QColor &c, int w, float step )
{
	drawLine ( QLineF ( effWin.left(), effWin.center().y(), effWin.right(), effWin.center().y()), w );	
	for ( float i=effWin.center().x() ; i< effWin.right(); i+=step )
	{
		drawLine ( QLineF ( i, effWin.center().y()-step, i , effWin.center().y()+step), c, w );
	}
	for ( float i=effWin.center().x() ; i> effWin.left(); i-=step )
	{
		drawLine ( QLineF ( i, effWin.center().y()-step, i , effWin.center().y()+step), c, w );
	}
	drawLine ( QLineF ( effWin.center().x(), effWin.top(), effWin.center().x(), effWin.bottom()), w );		
	for ( float i=effWin.center().y() ; i< effWin.bottom(); i+=step )
	{
		drawLine ( QLineF ( effWin.center().x()-step, i, effWin.center().x()+step, i), c, w );
	}
	for ( float i=effWin.center().y() ; i> effWin.top(); i-=step )
	{
		drawLine ( QLineF ( effWin.center().x()-step, i, effWin.center().y()+step, i), c, w );
	}
	drawText(QPointF(effWin.center().x()+1, effWin.top()), "0.5 m", effWin.width()/20, Qt::black);
}

void RCDraw::drawAxis ( const QColor &c, int w )
{
	static int step = 310; //ancho baldosa beta	

	for ( int i=step+visibleCenter(0); i< effWin.width()+visibleCenter(0); i+=step )
	{
		drawLine ( QLineF ( i, effWin.y()+visibleCenter(1), i, effWin.height()+visibleCenter(1) ), c, w );
	}
	for ( int i=-step+visibleCenter(0); i> effWin.x()+visibleCenter(0); i-=step )
	{
		drawLine ( QLineF ( i, effWin.y()+visibleCenter(1), i, effWin.height()+visibleCenter(1) ), c, w  );
	}
	for ( int i=step+visibleCenter(1); i< effWin.y()+visibleCenter(1); i+=step )
	{
		drawLine ( QLineF ( effWin.x()+visibleCenter(0), i, effWin.width()+visibleCenter(0), i ), c, w  );
	}
	for ( int i=-step+visibleCenter(1); i> effWin.height()+visibleCenter(1); i-=step )
	{
		drawLine ( QLineF ( effWin.x()+visibleCenter(0), i, effWin.width()+visibleCenter(0), i ), c, w  );
	}

	drawLine ( QLine ( rint(effWin.x()+visibleCenter(0)), 0, rint(effWin.x() + effWin.width()+visibleCenter(0)), 0 ), Qt::red, w*2 );
	drawLine ( QLine ( 0, effWin.y()+visibleCenter(1), 0,effWin.y() + effWin.height() + visibleCenter(1)), Qt::red, w*2 );
}

void RCDraw::drawCrossHair ( const QColor & c )
{
	drawLine ( QLine ( effWin.x(), effWin.height() /2, effWin.x() +effWin.width(), effWin.height() /2 ), c, 1 );
	drawLine ( QLine ( effWin.width() /2, effWin.y(), effWin.width() /2,effWin.y() +effWin.height() ), c, 1 );
}

void RCDraw::wheelEvent(QWheelEvent *event)
{
	const QRectF ini = effWin;

	float FF = zoomMul;
	if (event->delta()<0)
	{
		FF = 1./FF;
	}
	effWin.setWidth(FF*effWin.width());
	effWin.setHeight(FF*effWin.height());
	effWin.translate((1.-FF)/2.*ini.width(), (1.-FF)/2.*ini.height());
}


