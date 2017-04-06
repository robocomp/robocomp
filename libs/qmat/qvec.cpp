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
#ifdef COMPILE_IPP
// #include <ippWrapper.h>
#include <qmat/ippWrapper.h>
#endif

#include <qmat/qvec.h>
#include <qmat/qmat.h>

using namespace RMat;

/**
 * \brief constructor from (m,n) matrix into a m+n vector
 * @param matrix
 */
RMat::QVec::QVec(const RMat::QMat & matrix) : QVector<float>(matrix.nCols() * matrix.nRows())
{
	const int n = size();
	const float *matrixData = matrix.getReadData();

	for(int i = 0; i < n; i++)
		operator[](i) = matrixData[i];
}

// Vector - vector operators

/**
 * \brief Compares two vector for equal values element to element
 * @param vector
 * @return true if both are equal, false if not
 */
bool RMat::QVec::equals(const QVec & vector ) const
{
	if (size() != vector.size())
		return false;

	for (int i = 0; i < size(); i++)
		if (at(i) != vector[i])
			return false;

	return true;
}

/**
 * \brief operator that adds a vector to the current one
 * @param vector
 * @return the sum of both vectors
 */
QVec QVec::operator +(const QVec & vector) const
{
	Q_ASSERT( size() == vector.size() );
	QVec result(size());
	for (int i = 0; i < size(); i++)
		result[i] = at(i) + vector[i];
	return result;

}


/**
 * \brief operator that subtracts a vector from the current one
 * @param vector
 * @return the difference vector
 */
QVec QVec::operator -(const QVec & vector) const
{
	Q_ASSERT( size() == vector.size() );
	QVec result(size());
	for (int i = 0; i < size(); i++)
		result[i] = at(i) - vector[i];
	return result;

}

/**
 * \brief Computes de crossproduct of current 3-vector and a 3-vector parameter
 * @param vector 3-vector that operates on current vector
 * @return crossproduct
 */
QVec RMat::QVec::crossProduct(const QVec & vector) const
{
	Q_ASSERT(size() == vector.size());
	Q_ASSERT(size() == 3);
	const double    x1 = at(0), y1 = at(1), z1 = at(2), x2 = vector[0], y2 = vector[1], z2 = vector[2];
	QVec v(3);
	v[0] = -y2*z1 + y1*z2;
	v[1] = x2*z1 - x1*z2;
	v[2] = -x2*y1 + x1*y2;

	return v;
}

/**
 * \brief Dot product of operand with current vector
 * @param vector
 * @return dot product vector
 */
float QVec::dotProduct(const QVec &vector) const
{
	Q_ASSERT(size() == vector.size());

	double accum = 0;
	for (int i = 0; i < size(); i++)
		accum += at(i) * vector[i];
	return accum;
}

/**
 * \brief Element to element product
 * @param vector
 * @return product vector
 */
QVec  QVec::pointProduct(const QVec & vector) const
{
 	Q_ASSERT(size() == vector.size());

	QVec result = *this;
	for (int i = 0; i < size(); i++)
		result[i] *= vector[i];
	return result;
}

/**
 *\brief Computes anti-simmetric matrix A from currrent vector v so A*x = v cross x
 * @return cross product matrix
 */
QMat RMat::QVec::crossProductMatrix() const
{
	Q_ASSERT( size() == 3);

	QMat R (3,3);

	R(0,0) = 0.;
	R(0,1) = -operator[](2);
	R(0,2) = operator[](1);

	R(1,0) = operator[](2);
	R(1,1) = 0.;
	R(1,2) = -operator[](0);

	R(2,0) = -operator[](1);
	R(2,1) = operator[](0);
	R(2,2) = 0.;

	return R;
}

/**
 * \brief Injects a vector into current vector at defined offser
 * @param vector vector to be injected
 * @param offset displacement from zero position
 * @return injected vector
 */
const QVec & QVec::inject(const QVec &vector, const int offset)
{
	Q_ASSERT ( size() >= vector.size() + offset );

	for (int j=0; j<vector.size(); j++)
		operator()(j+offset) = vector.at(j);

	return *this;
}

/**
 * \brief Extracts a vector from current vector starting at firstIndex and ending at lastIndex
 * @param firstIndex initial position
 * @param lastIndex last position
 * @return extracted subvector
 */
QVec QVec::subVector(const int firstIndex, const int lastIndex) const
{
	Q_ASSERT(0 <= firstIndex);
	Q_ASSERT(firstIndex <= lastIndex);
	Q_ASSERT(lastIndex < this->size());

	int i,j;
	QVec result(lastIndex - firstIndex +1);
	for (i = firstIndex, j=0; i <= lastIndex; i++, j++)
		result[j] = operator[](i);
	return result;
}

// Vector - Escalar operations

/**
 * \brief Divides current vector by value
 * @param value dividing value
 * @return a copy of current vector divided by value
 */
QVec QVec::scalarDivision(const float value) const
{
	Q_ASSERT(value != 0);
	QVec result = *this;
	for (int i = 0; i < size(); i++)
	result[i] /= value;
	return result;
};

/**
 * \brief Multiplies current vector by value
 * @param value multiplying value
 * @return a copy of current vector multiplied by value
 */
QVec QVec::scalarMultiplication(const float value) const
{
	QVec result = *this;
	for (int i = 0; i < size(); i++)
		result[i] *= value;
	return result;
}

/**
 * \brief Add a scalar to curent vector
 * @param value
 * @return
 */
QVec RMat::QVec::operator +(const float value) const
{
	QVec result = *this;
	for (int i = 0; i < size(); i++)
		result[i] += value;
	return result;
}

/**
 * \brief Returns the minimun value and position of a vector
 * @return min value
 */
float RMat::QVec::min( int & pos) const
{
	float min = std::numeric_limits<float>::max();
	for (int i = 0; i < size(); i++)
		if ( operator[](i) < min)
		{
			min = operator[](i);
			pos = i;
		}
	return min;
}

/**
 * \brief Returns the minimun value and position of a vector
 * @return min value
 */
float RMat::QVec::min( ) const
{
	float min = std::numeric_limits<float>::max();
	for (int i = 0; i < size(); i++)
		if ( operator[](i) < min)
		{
			min = operator[](i);
		}
	return min;
}


/**
 * \brief Returns the minimun absolute value and position of a vector
 * @return min value
 */
float RMat::QVec::minAbs( int & pos) const
{
	float min = std::numeric_limits<float>::max();
	for (int i = 0; i < size(); i++)
	{
		float val = fabs(operator[](i));
		if ( val < min)
		{
			min = val;
			pos = i;
		}
	}
	return operator[](pos);
}

/**
 * \brief Returns the maximun value and position of a vector
 * @return max value
 */
float RMat::QVec::max( int & pos) const
{
	float max = std::numeric_limits<float>::min();
	for (int i = 0; i < size(); i++)
	{
		if ( operator[](i) > max)
		{
			max = operator[](i);
			pos = i;
		}
	}
	return max;
}

/**
 * \brief Returns the maximun value and position of a vector
 * @return max value
 */
float RMat::QVec::max( ) const
{
	float max = std::numeric_limits<float>::min();
	for (int i = 0; i < size(); i++)
	{
		if ( operator[](i) > max)
		{
			max = operator[](i);
		}
	}
	return max;
}

/**
 * \brief Returns the maximun value and position of a vector
 * @return max value
 */
float RMat::QVec::maxAbs( int & pos) const
{
	float max = std::numeric_limits<float>::min();
	for (int i = 0; i < size(); i++)
	{
		if ( fabs(operator[](i)) > max)
		{
			max = fabs(operator[](i));
			pos = i;
		}
	}
	return max;
}


// Vector - Matrix operations

/**
 * \brief Computes a QMat column matrix with current vector as its only column
 * @return column matrix
 */
QMat RMat::QVec::toColumnMatrix() const
{
	QMat result(size(), 1);
	result.setCol(0,*this);
	return result;
}

/**
 * \brief Computes a QMat row matrix with current vector as its only row
 * @return row matrix
 */
QMat RMat::QVec::toRowMatrix() const
{
	QMat result(1,size());
	result.setRow(0,*this);
	return result;
}

/**
 * \brief Multiplies a n-column vector times a m-row vector to generate a nxm matrix
 * @param vector vector to play the rol of row vector
 * @return resulting QMat matrix
 */
QMat QVec::externProduct(const QVec & vector) const
{
	Q_ASSERT(size() == vector.size());

	const int s = size();
	QMat C ( size(), vector.size() );
#ifdef COMPILE_IPP
	ippmMul_mm_32f ( getReadData(), s * sizeof ( float ), sizeof ( float ), s , s , vector.getReadData(), vector.size() * sizeof ( float ), sizeof ( float ), vector.size() , vector.size() , C.getWriteData(), vector.size() * sizeof ( float ), sizeof ( float ) );
#else
	for(int r=0;r<s;r++)
		for(int c=0;c<vector.size();c++)
			C(r,c) = this->operator()(r) * vector(c);
#endif
	return C;
}


//Utilities

/**
 * \brief Prints the content of a vector after the string s
 * @param s name to appear before vector values
 */
void RMat::QVec::print( const QString & s ) const
{
	std::cout << qPrintable ( s ) << "(" << size() << ")" << std::endl;
	std::cout << "[ " ;

	for ( int i=0; i < size() ; i++ )
	{
		std::cout << qPrintable ( QString ( "%1" ).arg ( /*this->operator()(i)*/ (*this)[i], -8, 'f', 6 ) ) << " ";
	fflush(stdout);
	}
	std::cout << "]" << std::endl;
}

/**
 * \brief Convertse QPointF variables from 2-vecs.
 * @return
 */
QPointF RMat::QVec::toQPointF() const
{
	Q_ASSERT( size() == 2);

	QPointF result;

	result.setX( operator[](0));
	result.setY( operator[](1));
	return result;
}

float RMat::QVec::distanceTo2DLine( const QVec & line ) const
{
  Q_ASSERT_X( size() == 2 and line.size() == 3, "QVec::distanceTo2DLine", "incorrect size of parameters");

   return fabs(line(0)*operator()(0) + line(1)*operator()(1) + line(2)) / sqrt(line(0)*line(0) + line(1)*line(1));

}

/**
 * @brief Computes de angle (-PI, PI) between this and endpoint p wrt the Y axis. If both points are the same, the method returns 0.
 *
 * @param p end point of segment
 * @return angle
 **/
float RMat::QVec::angleOf2DSegment( const QVec & p) const
{
	Q_ASSERT_X( size() == 2 and p.size()==2 , "QVec::angleOf2DSegment", "incorrect size of parameters");

	return atan2( p.x()-operator[](0), p.y()-operator[](1) );
}


bool RMat::QVec::isZero()
{
		for ( int i = 0; i < size(); i++ )
				if( this->operator[](i) != 0 )
					return false;
		return true;
}

/// Static methods


QVec RMat::QVec::vec6(float x, float y, float z, float rx, float ry, float rz)
{
	QVec R(6);
	R(0)=x;
	R(1)=y;
	R(2)=z;
	R(3)=rx;
	R(4)=ry;
	R(5)=rz;
	return R;
}

QVec RMat::QVec::vec6(QVec tv, QVec rv)
{
	QVec R(6);
	R(0)=tv(0);
	R(1)=tv(1);
	R(2)=tv(2);
	R(3)=rv(0);
	R(4)=rv(1);
	R(5)=rv(2);
	return R;
}



/**
 * \brief Static method that constructs a 4D-vector with given values
 * @param x
 * @param x
 * @param y
 * @param z
 * @param w
 * @return 4D-vector with x,y,z,w values initialized
 */
QVec RMat::QVec::vec4(float x, float y, float z, float w)
{
	QVec R(4);
	R(0)=x;
	R(1)=y;
	R(2)=z;
	R(3)=w;
	return R;
}

QVec RMat::QVec::vec4(RMat::QVec v3, float n)
{
	QVec R(4);
	R(0)=v3(0);
	R(1)=v3(1);
	R(2)=v3(2);
	R(3)=n;
	return R;
}

/**
 * \brief Static method that construcs a 3D-vector with given values
 * @param x
 * @param y
 * @param z
 * @return 3D-vector with x,y,z values initialized
 */
QVec RMat::QVec::vec3(float x, float y, float z)
{
	QVec R(3);
	R(0)=x;
	R(1)=y;
	R(2)=z;
	return R;
}

/**
 * \brief Static method that construct a 2D-vector with given values
 * @param x
 * @param y
 * @return 2D-vector with x,y values initialized
 */
QVec RMat::QVec::vec2(float x, float y)
{
	QVec R(2);
	R(0)=x;
	R(1)=y;
	return R;
}

/**
 * \brief Static method that construct a 2D-vector with given values
 * @param x
 * @param y
 * @return 1D-vector with x,y values initialized
 */
QVec RMat::QVec::vec1(float x)
{
	QVec R(1);
	R(0) = x;
	return R;
}


/**
 * \brief Static method that construc a vector of s element initialized to zero
 * @param s
 * @return zero valued s-vector
 */
QVec RMat::QVec::zeros(const int s )
{
	QVec result(s);
	return result;
}
//MAL
/**
 * \brief Static method that computes dim samples of a uniform distribution defined between min and max
 * @param dim number of samples
 * @param min min value
 * @param max max value
 * @return samples from uniform distribution
 */
// const QVec RMat::QVec::uniformVector(const int dim, const int min, const int max)
// {
// 	QVec res ( dim );
// 	T range = ( float )(abs(max)+abs(min)) / ( float ) std::numeric_limits<int>::max();
// 	for ( int i = 0; i < dim; i++ )
// 		res ( i ) =  range * rand() + min;
// 	return res;
// }

/**
 * \brief Static method that computes dim samples of a uniform distribution defined between min and max
 * @param dim number of samples
 * @param min min value
 * @param max max value
 * @return samples from uniform distribution
 */
const QVec RMat::QVec::uniformVector(const int dim, const float min, const float max)
{
	QVec res ( dim );
	float range = ( float )(max-min) / ( float ) RAND_MAX;
	for ( int i = 0; i < dim; i++ )
		res ( i ) =  range * rand() + min;
	return res;
}



/**
 * \brief Static method that computes gaussian samples from a gaussian distribution with moments mean and stdev
 * @param dim number of samples
 * @param mean mean of gaussian distribution
 * @param stdev standard deviation of gaussian distribution
 * @return dim-vec of gaussian values
 */
const QVec RMat::QVec::gaussianSamples(const int dim, const float mean, const float stdev)
{
	QVec R(dim);
	static unsigned int Seed = QTime::currentTime().msec();
#ifdef COMPILE_IPP
	ippsRandGauss_Direct_32f(R.getWriteData(), dim, mean, stdev, &Seed);
#else
	const gsl_rng_type * t;
	gsl_rng_env_setup();
	t = gsl_rng_default;
	gsl_rng *r = gsl_rng_alloc(t);
	gsl_rng_set(r,Seed);
	for (int i=0; i<dim; i++)
	{
		R(i) = gsl_ran_gaussian(r, stdev) + mean;
	}

#endif
  	return R;
}

// Static methods




/**
 * \brief Static method that computes a gaussian with 2*radius+1 elements and sigma stdev
 * @param radius half length of gaussian
 * @param sigma standard deviation
 * @return vector with discrete values of specified gaussian
 */
const QVec RMat::QVec::gaussianVector(const int radius, const float sigma)
{
	float kernelSum=0.0;
	QVec result(2*radius+1);
	for (int j=-radius;j<=radius;j++)
	{
		result[j+radius] = (float)expf(-(double)j*j/2.0/sigma/sigma);
		kernelSum += result[j+radius];
	}
	for (int j=-radius;j<=radius;j++)
		result[j+radius] /= kernelSum;

	return result;

}

/**
 * \brief Static method that computes a 3-vector from the 2-vector point adding a third coordinate initialised to 1
 * @param point input 2-vector
 * @return 3-vector in homogenous coordinates
 */
const QVec RMat::QVec::homogeneousCoordinates(const QPointF & point)
{
	QVec result(3, 1);

	result[0] = point.x(); result[1] = point.y();
	return result;

}

/**
 * Returns a vector with one additional dimension initialized to 1
 * @return n+1 vector
 */
QVec RMat::QVec::toHomogeneousCoordinates() const
{
	QVec res(size()+1);
	for(int i=0;i< size();i++)
		res(i) = this->operator[](i);
	res(size()) = 1.f;
	return res;
}

/**
 * Divide all elements by last row and removes it returning a n-1 dimension vector
 * @return n-1 vector
 */
QVec RMat::QVec::fromHomogeneousCoordinates() const
{
	Q_ASSERT( size() > 1);
	float final_l = this->operator[](size()-1);
// 	Q_ASSERT( fabs(final)>0);
	QVec res(size()-1);
	for(int i=0;i< res.size();i++)
		res(i) = this->operator[](i) / final_l;
	return res;
}

/**
 * \brief Computes the coefficients of the implicit line equation: y = mx + n passing by two points
 * from two points (x1,y1) e (x2,y2) satisfying  (y-y1)/(y2-y1) = (x-x1)/(x2-x1),
 * that solving for y gives:  y = x( (y2-y1)/(x2-x1) ) - x1( (y2-y1)/(x2-x1) ) + y1
 * @param p1 containing x1,y1
 * @param p2 containing x2,y2
 * @return a QVec vector of 2 dimensions with m and n
 */
QVec RMat::QVec::line2DImplicitCoefsFrom2Points(const QVec & p1, const QVec & p2)
{
	Q_ASSERT( p1.size() == 2 and p2.size() == 2 and p2(0)-p1(0) != 0 );
	QVec res(2);
	res(0) = (p2(1)-p1(1))/(p2(0)-p1(0)) ;
	res(1) =   p1(1) - p1(0) * ((p2(1)-p1(1))/(p2(0)-p1(0))) ;
	return res;
}

/**
 * \brief Computes the coefficients of the explicit line equation: Ax+By+C=0 passing by two points
 * from two points (x1,x2) e (y1,y2) satisfying  (x-x1)/v1 = (y-y1)/v2, being v1 and v2 the direction vectors of the line
 * that after some algebra gives: A=v2, B= -v1, C= v1y1-v2x2
 * @param p1 containing x1,y1
 * @param p2 containing x2,y2
 * @return a QVec vector of 3 dimensions with A, B and C
 */
QVec RMat::QVec::line2DExplicitCoefsFrom2Points(const QVec & p1, const QVec & p2)
{
	Q_ASSERT( p1.size() == 2 and p2.size() == 2);
	QVec res(3);
	//A
	res(0) = p2(1)-p1(1) ;
	//B
	res(1) = -(p2(0)-p1(0)) ;
	//C
	res(2) = -res(1)*p1(1) - res(0)*p1(0);
	return res;
}

std::ostream& 	operator << ( std::ostream &os, const RMat::QVec &vector )
{
	//os << "QVector(";
	for(int i=0; i<vector.size();i++)
	{
		os << vector[i];
		if(i<vector.size()-1)  //To avoid space ater the alst number
			os << " ";
	}
	//os << ")";
	return os;
};
std::istream& operator >> ( std::istream &is, RMat::QVec &vector )
	{
		vector.resize(3);  //ÑAPA

		 is >> vector[0];
		if((is.flags() & std::ios_base::skipws) == 0)
		{
			char whitespace;
			is >> whitespace;
		}
		is >> vector[1];
		if((is.flags() & std::ios_base::skipws) == 0)
		{
			char whitespace;
			is >> whitespace;
		}
		is >> vector[2];
		return is;
	}
