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

#include <qmat/qcamera.h>
#include <qmat/qvec.h>

using namespace RMat;

//Cam

/**
 * \brief Default constructor. 
 *
 * Initializes focus and offset to default values: \f$ fx = 200, fy=200, cx=160, cy=120 \f$
 */
Cam::Cam() :QMat ( 3,3 )
{
	//Deprecated
	focusX =  &( *this ) ( 0,0 );
	focusY =  &( *this ) ( 1,1 );
	centerX = &( *this ) ( 0,2 );
	centerY = &( *this ) ( 1,2 );

	//Default values
	focalX = 200; 
	focalY = 200;
	centroX = 160;
	centroY = 120;
	width = centroX *2 ;
	height = centroY *2 ;
	size = width * height ;
};

/**
 * \brief Copy constructor
 * @param c Camera matrix to copy from
 */
Cam::Cam ( const Cam & c ) : QMat ( c )
{
	this->focalX = c.getFocalX();
	this->focalY = c.getFocalY(); 
	this->width = c.getWidth();
	this->height = c.getHeight();
	this->size = c.getSize();

	//Deprecated
	focusX =  &( *this ) ( 0,0 );
	focusY =  &( *this ) ( 1,1 );
	centerX =  &( *this ) ( 0,2 );
	centerY =  &( *this ) ( 1,2 );
}
/**
 * \brief Explicit values constructor. Focuses and offsets are specified as parameters
 * @param Fx horizontal focus
 * @param Fy vertical focus
 * @param Ox horizontal position of image center in pixel coordinates
 * @param Oy vertical position of image center in pixel coordinates
 */
Cam::Cam ( T Fx, T Fy, T Ox, T Oy ) :QMat ( 3,3 )
{
	//Check for positive focus
// 	Q_ASSERT( Fx > 0 and Fy >0 and Ox >0 and Oy>0);

	focalX =  Fx;
	focalY =  Fy;
	centroX = Ox;
	centroY = Oy;
	width = centroX *2;
	height = centroY *2;
	size = width * height;
	
	//Insert focus as negative quantity
	this->operator()( 0,0 ) = Fx; this->operator()( 0,1 ) = 0.; this->operator()( 0,2 ) = Ox;
	this->operator()( 1,0 ) = 0.;this->operator()( 1,1 ) = -Fy;this->operator()( 1,2 ) = Oy;
	this->operator()( 2,0 ) = 0.;this->operator()( 2,1 ) = 0.;this->operator()( 2,2 ) =  1;

	//Deprecated
	focusX =  &( *this ) ( 0,0 );
	focusY =  &( *this ) ( 1,1 );
	centerX =  &( *this ) ( 0,2 );
	centerY =  &( *this ) ( 1,2 );
};


/**
 * \brief Destructor
 */
Cam::~Cam() {};

/**
 * \brief Sets values for focuses and offsets
 * @param Fx horizontal focus
 * @param Fy vertical focus
 * @param Ox horizontal position of image center in pixel coordinates
 * @param Oy vertical position of image center in pixel coordinates
 */
void Cam::set ( T Fx, T Fy, T Ox, T Oy )
{
	//Check for positive focus
// 	Q_ASSERT( Fx > 0 and Fy >0);

	focalX =  Fx;
	focalY =  Fy;
	centroX = Ox;
	centroY = Oy;
	width = centroX *2;
	height = centroY *2;
	size = width * height;
	
	this->operator()( 0,0 ) = Fx;	this->operator()( 0,1 ) = 0.;	this->operator()( 0,2 ) = Ox;
	this->operator()( 1,0 ) = 0.;this->operator()( 1,1 ) = -Fy;this->operator()( 1,2 ) = Oy;
	this->operator()( 2,0 ) = 0.;this->operator()( 2,1 ) = 0.;this->operator()( 2,2 ) =  1;

	//Deprecated
	focusX =  &( *this ) ( 0,0 );
	focusY =  &( *this ) ( 1,1 );
	centerX =  &( *this ) ( 0,2 );
	centerY =  &( *this ) ( 1,2 );
};

/**
* \brief Converts zero-centered image coordinates plus depth of a point to its 3D cartesian coordinates in ten camara reference frame
*
* Compute ... \n
* \f$ alfa = atan2( j, focusY) \f$ \n
* \f$ beta = atan2( i, focusX) \f$ \n
* \f$ proy = depth * cos(alfa) \f$ \n
* to obtain ... \n
* \f$ x = sin( beta ) \f$ \n
* \f$ y = proy * sin ( alfa ) \f$ \n
* \f$ z = proy * cos ( beta ) \f$ \n
* @param P \f$ p(0) = i p(1)= j p(2) = depth \f$
* @return QMat column matrix \f$ r(0) = x r(1)=y r(2) = z \f$
*/
QMat RMat::Cam::polar3DToCamera ( const QMat & p ) const
{
	QMat r(3);
	r(2) = ( p(2) / ( (p(0)*p(0) / (*focusX * *focusX)) + (p(1)*p(1)/ (*focusY * *focusY)) + 1.));
	r(0) = r(2) * p(0) / *focusX;
	r(1) = r(2) * p(1) / *focusY;

	return r;
}

/**
 * \brief Deprecated: Use vector version
*/
QMat RMat::Cam::project ( const QMat & p ) const
{
	qDebug() << "Deprecated";
	QMat loc = ( *this ) * p;
	QMat res ( 2 );
	if ( fabs ( loc ( 2 ) ) > 0.00001 )
	{
		res (0) = loc (0) / loc (2);
		res (1) = loc (1) / loc (2);
		return res;
	}
	else
	{
		qDebug() << "QCamera::project() -> Warning projected point at infinite";
		return res;
	}
}

/**
 * \brief Projects a 3D point p into this camera object. \f$ x = A*p \f$
 * @param p 3D point to be projected
 * @return A 2D vector of pixel coordinates (0,width ; 0,height)
 */
QVec RMat::Cam::project ( const QVec &p ) const
{
#ifdef DEBUG
	print("cam");
#endif
	QVec loc =  operator*(p);

	QVec res ( 3 );
	if ( fabs ( loc ( 2 ) ) != 0 )
	{
		res (0) = loc (0) / loc (2);
		res (1) = loc (1) / loc (2);
		res (2) = loc (2);
		return res;
	}
	else
	{
		qDebug() << "QCamera::project() -> Warning projected point at infinite";
		return res;
	}
}

/**
 * \brief Deprecated: Use vector version
*/
QMat RMat::Cam::getAngles(const QMat & p) const
{
	qWarning( "Deprecated");
	QMat r(2);
	r = toZeroCenter( p );
	r(0) = atan2( r(0), *focusX);
	r(1) = atan2( r(1), *focusY);	
	return r;
}


/**
 * \brief Computes the horizontal and vertical angles of optical rays going through image point p. Homogeneous version
 * @param p 2D image point in pixel coordinates defining the optical rays
 * @return Homogeneous 3-vector containing horizontal angle, vertical angle and a 1 in the third column.
 */
QVec Cam::getAnglesHomogeneous(const QVec & p) const
{
	Q_ASSERT( p.size() == 2 );
	QVec r(3);
	QVec c = toZeroCenter( p );
	r(0) = atan2( c(0), *focusX);
	r(1) = atan2( c(1), *focusY);	
	r(2) = 1.;
	return r;
}

/**
 * \brief Computes the horizontal and vertical angles of optical rays going through image point p
 * @param p 2D image point in pixel coordinates defining the optical rays
 * @return 2-vector containing horizontal angle and vertical angle
 */
QVec RMat::Cam::getAngles(const QVec & p) const
{
	Q_ASSERT( p.size() == 2 );
	QVec r(2);
	r = toZeroCenter( p );
	r(0) = atan2( r(0), *focusX);
	r(1) = atan2( r(1), *focusY);	
	return r;
}

/**
 * \brief Computes tangents of optical rays going through image point p. Homogeneous version
 * @param p 2D image point in pixel coordinates defining the optical rays
 * @return homogeneous 3-vector containing horizontal angle, vertical angle and a last 1.
 */
QVec Cam::getRayHomogeneous(const QVec & p) const
{
	QVec r = toZeroCenterHomogeneous( p );
	r(0) /= *focusX;
	r(1) /= *focusY;	
	return r;

}

/**
 * \brief Computes tangents of optical rays going through image point p. 
 * @param p 2D image point in pixel coordinates defining the optical rays
 * @return 2-vector containing horizontal angle and vertical angle
 */
QVec Cam::getRay(const QVec & p) const
{
	QVec r = toZeroCenter( p );
	r(0) /= *focusX;
	r(1) /= *focusY;	
	return r;

}

/**
 * \brief Deprecated: Use vector version
*/
QMat RMat::Cam::toZeroCenter( const QMat &p) const
{
	qDebug() << "Deprecated";
	QMat r(2);
	r(0) =  p(0) - (*centerX);
	r(1) =  p(1) - (*centerY); 
	return r;
}

/**
 * \brief Computes zero centered coordinates of a image point given in pize coordinates
 * @param p 2D point in pixel coordinates
 * @return 2D point in zero centered coordinates
 */
QVec Cam::toZeroCenter( const QVec &p) const
{
	Q_ASSERT( p.size() >= 2 );
	QVec r(2);
	r(0) =  p(0) - (*centerX);
	r(1) =  p(1) - (*centerY); 
	return r;
}

/**
 * \brief Computes zero centered coordinates of a image point given in pixel coordinates. Homogeneous version
 * @param p 2D point in pixel coordinates
 * @return 2D point in zero centered coordinates
 */
QVec Cam::toZeroCenterHomogeneous( const QVec &p) const
{
	Q_ASSERT( p.size() >= 2 );
	QVec r(3);
	r(0) =  p(0) - (*centerX);
	r(1) =  p(1) - (*centerY); 
	r(2) = 1.;
	return r;
}
/**
 * \brief Get focus x value 
 * @return float X focus value
 */
float Cam::getFocal() const
{
	return *focusX;
}
/**
 * \brief Get focus x value 
 * @return float X focus value
 */
float Cam::getFocalX() const
{ 
	return focalX;
}
/**
 * \brief Get focus y value 
 * @return float Y focus value
 */
float Cam::getFocalY() const
{ 
	return focalY;
}
/**
 * \brief Get height value 
 * @return float height focus value
 */
int Cam::getHeight() const
{ 
	return height; 
}
/**
 * \brief Get size value 
 * @return float size value
 */
int Cam::getSize() const
{
	return size;
}
/**
 * \brief Get width value 
 * @return float width value
 */
int Cam::getWidth() const
{
	return width;
}
/**
 * \brief Set focus x and y to specific same value
 * @param f value to set
 */
void Cam::setFocal( const int f) const
{ 
	*focusX = f;
	*focusY = f;
}
/**
 * \brief Set focus x to specific value
 * @param f value to set
 */
void Cam::setFocalX( const int fx)
{ 
	focalX = fx;
}
/**
 * \brief Set focus y to specific value
 * @param f value to set
 */
void Cam::setFocalY( const int fy)
{ 
	focalY = fy;
}
/**
 * \brief Set width and height
 * @param w width value to set
 * @param h heigth value to set
 */
void Cam::setSize(int w, int h)
{ 
	width = w;
	height = h; 
	size = w*h; 
}
