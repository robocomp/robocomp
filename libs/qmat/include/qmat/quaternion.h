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

/*      Copyright (C) 2007, 2008, 2009. PARP Research Group.
*      <http://perception.inf.um.es>
*      University of Murcia, Spain.
*
*      This file is part of the QVision library.
*
*      QVision is free software: you can redistribute it and/or modify
*      it under the terms of the GNU Lesser General Public License as
*      published by the Free Software Foundation, version 3 of the License.
*
*      QVision is distributed in the hope that it will be useful,
*      but WITHOUT ANY WARRANTY; without even the implied warranty of
*      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*      GNU Lesser General Public License for more details.
*
*      You should have received a copy of the GNU Lesser General Public
*      License along with QVision. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef QQUATERNION_H
#define QQUATERNION_H

#include <math.h>
#include <iostream>
#include <QString>


#include <qmat/qvec.h>
#include <qmat/qmat.h>


namespace RMat
{

	class QQuaternion: public QVec
	{
		public:
			// Constructors

			QQuaternion();
			QQuaternion(QVec direction, float phi);
			QQuaternion(const double q1, const double q2, const double q3, const double q4);
			QQuaternion(const QMat matrix);
			QQuaternion(const QQuaternion &quaternion): QVec(quaternion){}
			
			QQuaternion operator*(const QQuaternion &quaternion) const            { return quaternionProduct(quaternion);};
			static QQuaternion trackball(float p1x, float p1y, float p2x, float p2y);
			QQuaternion quaternionProduct(const QQuaternion &quaternion) const;

		private:
			QQuaternion normalizeQuaternion() const;
	};

std::ostream& operator << ( std::ostream &os, const QQuaternion &quaternion );
}
#endif
