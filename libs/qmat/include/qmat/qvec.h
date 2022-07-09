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
#ifndef QVEC_H
#define QVEC_H

#include <math.h>
#include <iostream>
#include <QtCore>
#include <vector>
#include <limits>
#include <qmat/qmat.h>


#ifdef PYTHON_BINDINGS_SUPPORT
	#include <boost/python.hpp>
	#include <boost/python/list.hpp>
#endif

namespace RMat
{
	/*!
	 *\brief Defines a vector class and standard algebraic operations on it.
	 *@author Robolab
	 *
	 * QVec derives from the Qt dynamic array QVector. It implements standard vector operations and also external operations that produce QMat elements.
	 *
	 *
	*/
	class QVec : public QVector<T>
	{
	public:
		QVec() : QVector()                                                                {}
		QVec(const int size) : QVector<T>(size)                                              {}
		QVec(const int size, const T defaultValue) : QVector(size, defaultValue)      {}
		QVec(const RMat::QMat & matrix);
		QVec(const QVec & v) : QVector(v)                                                 {}
		QVec(QVec&& v) noexcept : QVector(std::move(v)) 								  {}
		QVec(const QVector &vector): QVector(vector)                                      {}
		QVec(const std::vector<T> &vector): QVector( vector.begin(), vector.end() )        {}
#ifdef PYTHON_BINDINGS_SUPPORT
		QVec(const boost::python::list &v): QVector(QVector::fromStdVector(to_std_vector_QVec(v))) {}
#endif
		QVec(const QPoint &point): QVector(2)                   { operator[](0) = point.x(); operator[](1) = point.y(); }
		QVec(const QPointF &point): QVector(2)                  { operator[](0) = point.x(); operator[](1) = point.y(); }

		QVec& operator=(QVec& other) = default;
		QVec& operator=(const QVec& other) = default;
		QVec& operator=(QVec&& other){ this->swap(other); return *this;};
		QVec& operator=(const QVec&& other){ auto tmp = other; this->swap(tmp); return *this;};
		
		const T *getReadData() const                        { return constData(); }
		T *getWriteData()                                   { return data(); }
		std::vector<T> toVector3()                          { Q_ASSERT(size()==3); return std::vector<T>{operator[](0),operator[](1),operator[](2)};};

		// Vector - Vector operators
		inline T & operator()(const int ind)                { return operator[](ind);}
		inline T operator()(const int ind) const            { return operator[](ind);}
		inline void setItem(const int ind, const T value)   { operator[](ind) = value; }
		inline T getItem(const int ind) const               { return operator[](ind); }
		T operator*(const QVec &vector) const               { return dotProduct(vector); }
		QVec operator^(const QVec &vector) const            { return crossProduct(vector); }
		QVec operator+(const QVec &vector) const;
		QVec operator+(const T value) const;
		QVec operator-(const QVec &vector) const;
		QVec operator*(const T value) const                 { return scalarMultiplication(value); }
		QVec operator/(const T value) const                 { return scalarDivision(value); }
		QVec operator& ( const QVec &vector ) const             { return pointProduct(vector); }
		bool operator==(const QVec &vector) const               { return equals(vector); }
		QVec & operator+=(const QVec &vector)                   { return (*this = this->operator+(vector)); }
		QVec & operator-=(const QVec &vector)                   { return (*this = this->operator-(vector)); }

		// Vector - Matrix operators
		QMat operator*(const QMat &matrix) const                { return this->toRowMatrix()*matrix; }

		QMat operator|(const QVec &vector) const                { return externProduct(vector); }

		operator QPointF() const //Returns a QPointF type from current vector
		{
			Q_ASSERT(size() > 1);
			Q_ASSERT(size() < 4);
			if (size() == 2)
				return QPointF(operator[](0), operator[](1));
			else
				return QPointF(operator[](0)/operator[](2), operator[](1)/operator[](2));
		}

		//  Vector - Vector methods
		void set(const T value)                              { fill( value ); }
		QVec subVector(const int firstIndex, const int lastIndex) const; //Returns a subvector starting at firstIndex and ending at lastIndex, both included
		QVec scalarDivision(const T value) const;                        //Divides all elements of the vector by value
		QVec scalarMultiplication(const T value) const;                  //Multiplies all elements of the vector by value
		const QVec & inject(const QVec &vector, const int offset);
		QVec pointProduct(const QVec &vector) const;
		QVec normalize() const                                   { return operator/(norm2()); }
		QVec crossProduct(const QVec &vector) const;
		bool equals(const QVec &vector, T epsilon=1) const;
		QVec toHomogeneousCoordinates() const;
		QVec fromHomogeneousCoordinates() const;

		// Vector - Escalar methods
		T norm2() const                                      { return sqrt(*this * *this); }
		T max( int & pos ) const;
		T max() const;
		T maxP() const { return max(); }
		T min( int & pos) const;
		T min( ) const;
		T minP() const { return min(); }
		T minAbs( int & pos) const;
		T maxAbs( int & pos) const;
// 		T mean() const;
// 		T variance() const;
		T dotProduct(const QVec &vector) const;
		inline T x() const       { Q_ASSERT(size()>0); return this->operator[](0); }
		inline T y() const       { Q_ASSERT(size()>1); return this->operator[](1); }
		inline T z() const       { Q_ASSERT(size()>2); return this->operator[](2); }
		inline T rx() const      { Q_ASSERT(size()>3); return this->operator[](3); }
		inline T ry() const      { Q_ASSERT(size()>4); return this->operator[](4); }
		inline T rz() const      { Q_ASSERT(size()>5); return this->operator[](5); }
		inline T alfa() const    { Q_ASSERT(size()>2); return this->operator[](2); }
		inline T alpha() const   { Q_ASSERT(size()>2); return this->operator[](2); }

		// Vector - Matrix methods
		QMat externProduct(const QVec &vector) const;
		QMat crossProductMatrix() const;
		QMat toRowMatrix() const;
		QMat toColumnMatrix() const;
		bool isZero();

		// Vector - QPoint
		QPointF toQPointF() const;

		// Print
		void print(const QString & s) const;
		void prints(const std::string & s) const { print(QString::fromStdString(s)); }

		//Geometry
		T distanceTo2DLine( const QVec & line ) const;
		T angleOf2DSegment( const QVec & p12) const;

		//Static methods
		static QVec vec6(T x=0., T y=0., T z=0., T rx=0., T ry=0., T rz=0.);
		static QVec vec6(QVec tv, QVec rv);
		static QVec vec4(T x=0., T y=0., T z=0., T w=0.);
		static QVec vec4(QVec v3, T n=0.);
		static QVec vec3(T x=0., T y=0., T z=0.);
		static QVec vec2(T x=0., T y=0.);
		static QVec vec1(T x=0.);
		static const QVec gaussianVector(const int radius, const T sigma);
// 		static const QVec uniformVector(const int dim , const int min, const int max);
		static const QVec uniformVector(const int dim , const T min, const T max);
		static const QVec gaussianSamples(const int dim, const T center, const T sigma);
		static const QVec homogeneousCoordinates(const QPointF &point);
		static QVec zeros(const int s);
		static QVec line2DImplicitCoefsFrom2Points(const QVec & p1, const QVec & p2);
		static QVec line2DExplicitCoefsFrom2Points(const QVec & p1, const QVec & p2);

	};

};

std::ostream& operator << ( std::ostream &os, const RMat::QVec &vector );
std::istream& operator >> ( std::istream &is, RMat::QVec &vector );

#endif
