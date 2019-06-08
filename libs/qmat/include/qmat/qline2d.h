/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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


#ifndef QLINE2D_H
#define QLINE2D_H

#include <QtCore>
#include <limits>
#include <qmat/qvec.h>

using namespace RMat;

class QLine2D : public QVec
{
	public:
		QLine2D() : QVec(3,0.f) {};               
		QLine2D(const QVec &p1, const QVec &p2);
		QLine2D(T x1, T y1, T x2, T y2);
		QLine2D(const QLine2D& other): QVec(3)				{setA(other.A());setB( other.B()); setC( other.C());};
		QLine2D(const QVec &dirVector, T x, T y);
		~QLine2D()											{};
		virtual bool operator==(const QLine2D& other) const;
		T perpendicularDistanceToPoint(const QVec &point);
		T signedAngleWithLine2D(const QLine2D &line);
		void print(const QString &p) 						{qDebug() << p << A() << B() << C();};
		inline T A() const 									{return (*this)[0];};
		inline T B() const 									{return (*this)[1];};
		inline T C() const 									{return (*this)[2];};
		inline void setA(float a)  							{(*this)[0] = a;};	
		inline void setB(float b)  							{(*this)[1] = b;};	
		inline void setC(float c)  							{(*this)[2] = c;};	
		QLine2D getPerpendicularLineThroughPoint(const QVec &point);  //2D point
		QVec getDirectionVector() const 					{ return QVec::vec2(-B(),A());};;
		QVec getNormalizedDirectionVector() const 			{ return QVec::vec2(-B(),A()).normalize();};;
		QVec getPerpendicularVector() const					{ return QVec::vec2(A(),B());};
		QVec intersectionPoint(const QLine2D &l);
		QLine2D getNormalLineThroughOrigin();
		QVec getIntersectionPointOfNormalThroughOrigin();
		QVec getNormalForOSGLineDraw();
		QLine2D getPlus45DegreesLinePassingThroughPoint(const QVec &point);
		T getAngleWithZAxis();
		QVec pointAlongLineStartingAtP1AtLanda(const QVec &p1, float landa);
};

std::ostream& operator << ( std::ostream &os, const RMat::QVec &vector );

#endif // LINE2D_H
















