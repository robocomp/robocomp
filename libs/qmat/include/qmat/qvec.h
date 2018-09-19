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
	class QVec : public QVector<float>
	{
	public:
		QVec() : QVector()                                                                {}
		QVec(const int size) : QVector<float>(size)                                              {}
		QVec(const int size, const float defaultValue) : QVector(size, defaultValue)      {}
		QVec(const RMat::QMat & matrix);
		QVec(const QVec & v) : QVector(v)                                                 {}
		QVec(QVec&& v) noexcept : QVector(std::move(v)) 								  {}
		QVec(const QVector &vector): QVector(vector)                                      {}
		QVec(const std::vector<float> &vector): QVector(QVector::fromStdVector( vector ))        {}
#ifdef PYTHON_BINDINGS_SUPPORT
		QVec(const boost::python::list &v): QVector(QVector::fromStdVector(to_std_vector_QVec(v))) {}
#endif
		QVec(const QPoint &point): QVector(2)                   { operator[](0) = point.x(); operator[](1) = point.y(); }
		QVec(const QPointF &point): QVector(2)                  { operator[](0) = point.x(); operator[](1) = point.y(); }

		QVec& operator=(QVec& other) = default;
		QVec& operator=(const QVec& other) = default;
		QVec& operator=(QVec&& other){ this->swap(other); return *this;};
		QVec& operator=(const QVec&& other){ auto tmp = other; this->swap(tmp); return *this;};
		
		const float *getReadData() const                        { return constData(); }
		float *getWriteData()                                   { return data(); }

		// Vector - Vector operators
		inline float & operator()(const int ind)                { return operator[](ind);}
		inline float operator()(const int ind) const            { return operator[](ind);}
		inline void setItem(const int ind, const float value)   { operator[](ind) = value; }
		inline float getItem(const int ind) const               { return operator[](ind); }
		float operator*(const QVec &vector) const               { return dotProduct(vector); }
		QVec operator^(const QVec &vector) const                { return crossProduct(vector); }
		QVec operator+(const QVec &vector) const;
		QVec operator+(const float value) const;
		QVec operator-(const QVec &vector) const;
		QVec operator*(const float value) const                 { return scalarMultiplication(value); }
		QVec operator/(const float value) const                 { return scalarDivision(value); }
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
		void set(const float value)                              { fill( value ); }
		QVec subVector(const int firstIndex, const int lastIndex) const; //Returns a subvector starting at firstIndex and ending at lastIndex, both included
		QVec scalarDivision(const float value) const;                        //Divides all elements of the vector by value
		QVec scalarMultiplication(const float value) const;                  //Multiplies all elements of the vector by value
		const QVec & inject(const QVec &vector, const int offset);
		QVec pointProduct(const QVec &vector) const;
		QVec normalize() const                                   { return operator/(norm2()); }
		QVec crossProduct(const QVec &vector) const;
		bool equals(const QVec &vector) const;
		QVec toHomogeneousCoordinates() const;
		QVec fromHomogeneousCoordinates() const;

		// Vector - Escalar methods
		float norm2() const                                      { return sqrt(*this * *this); }
		float max( int & pos ) const;
		float max() const;
		float maxP() const { return max(); }
		float min( int & pos) const;
		float min( ) const;
		float minP() const { return min(); }
		float minAbs( int & pos) const;
		float maxAbs( int & pos) const;
// 		float mean() const;
// 		float variance() const;
		float dotProduct(const QVec &vector) const;
		inline float x() const       { Q_ASSERT(size()>0); return this->operator[](0); }
		inline float y() const       { Q_ASSERT(size()>1); return this->operator[](1); }
		inline float z() const       { Q_ASSERT(size()>2); return this->operator[](2); }
		inline float rx() const      { Q_ASSERT(size()>3); return this->operator[](3); }
		inline float ry() const      { Q_ASSERT(size()>4); return this->operator[](4); }
		inline float rz() const      { Q_ASSERT(size()>5); return this->operator[](5); }
		inline float alfa() const    { Q_ASSERT(size()>2); return this->operator[](2); }
		inline float alpha() const   { Q_ASSERT(size()>2); return this->operator[](2); }

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
		float distanceTo2DLine( const QVec & line ) const;
		float angleOf2DSegment( const QVec & p12) const;

		//Static methods
		static QVec vec6(float x=0., float y=0., float z=0., float rx=0., float ry=0., float rz=0.);
		static QVec vec6(QVec tv, QVec rv);
		static QVec vec4(float x=0., float y=0., float z=0., float w=0.);
		static QVec vec4(QVec v3, float n=0.);
		static QVec vec3(float x=0., float y=0., float z=0.);
		static QVec vec2(float x=0., float y=0.);
		static QVec vec1(float x=0.);
		static const QVec gaussianVector(const int radius, const float sigma);
// 		static const QVec uniformVector(const int dim , const int min, const int max);
		static const QVec uniformVector(const int dim , const float min, const float max);
		static const QVec gaussianSamples(const int dim, const float center, const float sigma);
		static const QVec homogeneousCoordinates(const QPointF &point);
		static QVec zeros(const int s);
		static QVec line2DImplicitCoefsFrom2Points(const QVec & p1, const QVec & p2);
		static QVec line2DExplicitCoefsFrom2Points(const QVec & p1, const QVec & p2);

	};

};

std::ostream& operator << ( std::ostream &os, const RMat::QVec &vector );
std::istream& operator >> ( std::istream &is, RMat::QVec &vector );

#endif
