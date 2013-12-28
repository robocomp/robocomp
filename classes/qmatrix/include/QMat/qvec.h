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
#include <QMat/qmat.h>


namespace RMat
{
	typedef float T;

	/*!
	 *\brief Defines a vector class and standard algebraic operations on it.
	 *@author Robolab
	 *
	 * QVec derives from the Qt dynamic array QVector. It implements standard vector operations and also external operations that produce QMat elements.
	 *
	 *
	*/
	class QVec : public QVector< T >
	{
	public:
		QVec() : QVector<T>()                                                                       {}
		QVec(const int size, const T defaultValue = 0) : QVector< T >(size, defaultValue)           {}
		QVec(const RMat::QMat & matrix);
		QVec(const QVec & v) : QVector<T>(v)                                                        {}
		QVec(const QVector<T> &vector): QVector<T>(vector)                                          {}
		QVec(const std::vector<T> &vector): QVector<T>(QVector<T>::fromStdVector( vector ))         {}
		QVec(const QPoint &point): QVector< T >(2)          { operator[](0) = point.x(); operator[](1) = point.y(); }
		QVec(const QPointF &point): QVector< T >(2)         { operator[](0) = point.x(); operator[](1) = point.y(); }

		const T *getReadData() const                        { return constData(); }
		T *getWriteData()                                   { return data(); }

		// Vector - Vector operators
		inline T & operator()(const int ind)                { return operator[](ind);}
		inline T operator()(const int ind) const            { return operator[](ind);}
		T operator*(const QVec &vector) const               { return dotProduct(vector); }
		QVec operator^(const QVec &vector) const            { return crossProduct(vector); }
		QVec operator+(const QVec &vector) const;
		QVec operator+(const T value) const;
		QVec operator-(const QVec &vector) const;
		QVec operator*(const T value) const                 { return scalarMultiplication(value); }
		QVec operator/(const T value) const                 { return scalarDivision(value); }
		QVec operator& ( const QVec &vector ) const         { return pointProduct(vector); }
		bool operator==(const QVec &vector) const           { return equals(vector); }
		QVec & operator+=(const QVec &vector)               { return (*this = this->operator+(vector)); }
		QVec & operator-=(const QVec &vector)               { return (*this = this->operator-(vector)); }

		// Vector - Matrix operators
		QMat operator*(const QMat &matrix) const            { return this->toRowMatrix()*matrix; }

		QMat operator|(const QVec &vector) const            { return externProduct(vector); }

		operator QPointF() const
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
		QVec subVector(const int firstIndex, const int lastIndex);
		QVec scalarDivision(const T value) const;
		QVec scalarMultiplication(const T value) const;
		const QVec & inject(const QVec &vector, const int offset);
		QVec pointProduct(const QVec &vector) const;
		QVec normalize() const                               { return operator/(norm2()); }
		QVec crossProduct(const QVec &vector) const;
		QVec substract(const QVec &vector) const;
		bool equals(const QVec &vector) const;
		QVec toHomogeneousCoordinates() const;
		QVec fromHomogeneousCoordinates() const;

		// Vector - Escalar methods
		T norm2() const                                      { return sqrt(*this * *this); }
		T max( int & pos) const;
		T min( int & pos) const;
		T minAbs( int & pos) const;
		T maxAbs( int & pos) const;
		T mean() const;
		T variance() const;
		T dotProduct(const QVec &vector) const;
		inline T x() const                                   { return this->operator[](0); }
		inline T y() const                                   { return this->operator[](1); }
		inline T z() const                                   { return this->operator[](2); }
		inline T alfa() const                                { return this->operator[](2); }
		inline T alpha() const                                { return this->operator[](2); }

		// Vector - Matrix methods
		QMat externProduct(const QVec &vector) const;
		QMat crossProductMatrix() const;
		QMat toRowMatrix() const;
		QMat toColumnMatrix() const;

		// Vector - QPoint
		QPointF toQPointF() const;

		// Print
		void print(const QString & s) const;

		//Geometry
		T distanceTo2DLine( const QVec & line ) const;
		T angleOf2DSegment( const QVec & p12) const;

		//Static methods
		static QVec vec4(T x=0., T y=0., T z=0., T w=0.);
		static QVec vec4(QVec v3, T n=0.);
		static QVec vec3(T x=0., T y=0., T z=0.);
		static QVec vec2(T x=0., T y=0.);
		static QVec vec1(T x=0.);
		static const QVec gaussianVector(const int radius, const T sigma);
// 		static const QVec uniformVector(const int dim , const int min, const int max);
		static const QVec uniformVector(const int dim , const float min, const float max);
		static const QVec gaussianSamples(const int dim, const T center, const T sigma);
		static const QVec homogeneousCoordinates(const QPointF &point);
		static QVec zeros(const int s);
		static QVec line2DImplicitCoefsFrom2Points(const QVec & p1, const QVec & p2);
		static QVec line2DExplicitCoefsFrom2Points(const QVec & p1, const QVec & p2);
	};
};

std::ostream& operator << ( std::ostream &os, const RMat::QVec &vector );

#endif

