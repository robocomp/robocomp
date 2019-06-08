
#include <qmat/qline2d.h>
/*
QLine2D::QLine2D()
{
}
*/

/**
 * @brief Normalized constructor of a 2D line from a direction vector and a point through the line
 * 
 * @param dirVector 2D QVector
 * @param x ...
 * @param y ...
 */
QLine2D::QLine2D(const QVec &dirVector, T x, T y) : QVec(3)
{
	Q_ASSERT_X(dirVector.size()  == 2, "QLine2D direction vector constructor", "dirvector size must be 2");

	QVec dn = dirVector; /*.normalize();*/
// 	A = -dn[1];
// 	B = dn[0];
// 	C = -(A*x + B*y);
	setA( -dn[1] );
	setB( dn[0] );
	setC( A()*x + B()*y );
}

QLine2D::QLine2D(const QVec &p1, const QVec &p2): QVec(3)
{
	Q_ASSERT_X( ( p1.size()==2 and p2.size()==2 ) or ( p1.size()==3 and p2.size()==3 ), "QLine2D 2 points constructor", "size of p1 and p1 must be 2 or 3" ) ;
	if( p1.size()==2 and  p2.size()==2) 
	{
		setA( -(p2.y() - p1.y()) );
		setB( p2.x() - p1.x() );
		setC( -(A()*p1.x() + B()*p1.y()) );
	}
	else if ( p1.size()==3 and  p2.size()==3) //Assume a Robocomp 3D rep with Y pointg upwards
	{
		setA( -(p2.z() - p1.z()) );
		setB(  p2.x() - p1.x() ) ;
		setC( -(A()*p1.x() + B()*p1.z()) );
	}
	Q_ASSERT_X(fabs(A())>0 or fabs(B())>0, "QLine2D 2 points constructor", "|A| or |B| must be > 0" );
	float m = sqrt(A()*A()+B()*B());
	setA(  A()/m );
	setB(  B()/m );
	setC(  C()/m );
}

QLine2D::QLine2D(T x1, T y1, T x2, T y2): QVec(3)
{
	setA( -(y2 - y1) );
	setB( x2 - x1 );
	setC( -(A()*x1 + B()*y1) );
	
	Q_ASSERT_X(fabs(A())>0 or fabs(B())>0, "QLine2D 2 points constructor", "|A| or |B| must be > 0" );
	float m = sqrt(A()*A()+B()*B());
	setA(  A()/m );
	setB(  B()/m );
	setC(  C()/m );
}

/**
 * @brief Equality operator
 * 
 * @param other ...
 * @return bool
 */
bool QLine2D::operator==(const QLine2D& other) const
{
	return (A()==other.A() and B()==other.B() and C()==other.C());
}


/**
 * @brief Computes the perpendicular distante from the line to point.
 * Accepts 2D and 3D points giving that it is assumed that line is on the (X,Z) plane
 * If a 2D point is passed, first and second coordinates are used
 * If a 3D point is passed, first and third coordiantes are used
 * @param point ...
 * @return RMat::T
 */
RMat::T QLine2D::perpendicularDistanceToPoint(const RMat::QVec& point)
{
	if( point.size() == 2)
		return (A()*point.x() + B()*point.y() + C()) / sqrt(A()*A() + B()*B());
	else if( point.size() == 3)
		return (A()*point.x() + B()*point.z() + C()) / sqrt(A()*A() + B()*B());		
	else
	{
		qDebug() << __FILE__ << __FUNCTION__ << "Size of vector not 2 or 3";
		qFatal("Aborting");
		return 0;
	}
}

/**
 * @brief Computes the signed angle between this and line. Then angle is computed between -PI and PI
 * 
 * @param line ...
 * @return RMat::T
 */
T QLine2D::signedAngleWithLine2D(const QLine2D& line)
{
	Q_ASSERT(fabs(line.A())>0 or fabs(line.B())>0);
 	QVec dirA = this->getNormalizedDirectionVector();
 	QVec dirB = line.getNormalizedDirectionVector();
	QVec A3 = QVec::vec3(dirA.x(),0,dirA.y());
	QVec B3 = QVec::vec3(dirB.x(),0,dirB.y());
	T cosAng = dirA * dirB;
	QVec cross = A3^B3; 	// cross product to set the sign of the angle, checking the perpendicular Y coordinate 
	float angle = acos(cosAng);
	if ( cross.y() < 0)              
		angle = -angle;
	return angle;
}

QLine2D QLine2D::getPerpendicularLineThroughPoint(const QVec& point)
{
		Q_ASSERT_X(point.size()==2,"QLine2D::perpendicularDistanceToPoint", "Point size != 2");
 		return QLine2D( this->getPerpendicularVector(), point.x(), point.y());
}

QVec QLine2D::intersectionPoint(const QLine2D& l)
{
	Q_ASSERT_X(fabs(l.A()) > 0 or fabs(l.B())>0, "QLine2D::intersectionPoint", "|A| or |B| must be > 0");
	QMat d(2,2);
	d(0,0) = A(); d(0,1) = B();
	d(1,0) = l.A(); d(1,1) = l.B();
	QVec b = QVec::vec2(-C(), -l.C());
	return (d.invert()*b);
}

QLine2D QLine2D::getNormalLineThroughOrigin()
{
	return getPerpendicularLineThroughPoint(QVec::zeros(2));
}

QVec QLine2D::getIntersectionPointOfNormalThroughOrigin()
{
	return this->intersectionPoint( this->getNormalLineThroughOrigin() );
}

QLine2D QLine2D::getPlus45DegreesLinePassingThroughPoint(const QVec& point)
{
	float angle = this->getAngleWithZAxis();
	//qDebug() << "anglewithZaxis" << angle;
	angle = angle + M_PI/4.;
	QVec v = QVec::vec2(sin(angle),cos(angle));
	v.print("v");
	//point.print("point");
	QLine2D l( v, point.x(), point.z() );
	//l.print("l");
	return l;
	
}

T QLine2D::getAngleWithZAxis()
{
	QVec d = this->getDirectionVector();
	return atan2( d.x(), d.y() );
}

QVec QLine2D::pointAlongLineStartingAtP1AtLanda(const QVec &p1, float landa)
{
		QVec d = this->getDirectionVector();
		QVec dd = QVec::vec3(d.x(),0,d.y());
		return p1 + (d * landa);
}

QVec QLine2D::getNormalForOSGLineDraw()
{
	QVec p = getIntersectionPointOfNormalThroughOrigin();  //2 vector
	if( p.isZero() == true )  //both intersect at 0,0. Return a fake small vector normal to the line at origin
	{
		//qDebug() << "QLine2D::getNormalForOSGLineDraw - shit Intersection point is zero";
		return QVec::vec3((this->getPerpendicularVector() / 1000.f).x(), 0, (this->getPerpendicularVector() / 1000).y());
	}
	else 
		return QVec::vec3(p.x(),0, p.y());
}

