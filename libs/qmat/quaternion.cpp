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

/*
*      Copyright (C) 2007, 2008, 2009. PARP Research Group.
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


#include <qmat/quaternion.h>

using namespace RMat;

// Constructors
QQuaternion::QQuaternion(): QVec(4)
{
	set(0);
	operator[](3) = 1;
}

QQuaternion::QQuaternion(QVec a, float phi): QVec(4)
{
	QVec a_normal = a * (T)(sin(phi/2.0) / a.norm2());

	operator[](0) = a_normal[0];
	operator[](1) = a_normal[1];
	operator[](2) = a_normal[2];
	operator[](3) = cos(phi/2.0);
}

QQuaternion::QQuaternion(const QMat matrix): QVec(4)
{
	Q_ASSERT(matrix.nCols() == 3);
	Q_ASSERT(matrix.nFils() == 3);

	operator[](3) = sqrt(1 + matrix(0,0) + matrix(1,1) + matrix(2,2)) / 2;
	operator[](0) = (matrix(2,1) - matrix(1,2)) / (4*operator[](3));
	operator[](1) = (matrix(0,2) - matrix(2,0)) / (4*operator[](3));
	operator[](2) = (matrix(1,0) - matrix(0,1)) / (4*operator[](3));
}

QQuaternion::QQuaternion(const double q1, const double q2, const double q3, const double q4): QVec(4)
{
	operator[](0) = q1;
	operator[](1) = q2;
	operator[](2) = q3;
	operator[](3) = q4;
}

QQuaternion QQuaternion::normalizeQuaternion() const
{
	QQuaternion q = *this;

	double size = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
	for (int i = 0; i < 4; i++)
	q[i] /= size;

	return q;
}

QQuaternion QQuaternion::quaternionProduct(const QQuaternion &quaternion) const
{
	Q_ASSERT(size() == 4);
	Q_ASSERT(quaternion.size() == 4);

	QQuaternion q1Vector = *this, q2Vector = quaternion;

	QVec t1Vector = q1Vector.subVector(0,2);
	QVec t2Vector = q2Vector.subVector(0,2);
	QVec t3Vector = t2Vector ^ t1Vector;
	QVec tfVector = t1Vector * q2Vector[3] + t2Vector * q1Vector[3] + t3Vector;

	QQuaternion destQuat;

	destQuat[0] = tfVector[0];
	destQuat[1] = tfVector[1];
	destQuat[2] = tfVector[2];
	destQuat[3] = q1Vector[3] * q2Vector[3] - t1Vector * t2Vector;

	return destQuat.normalizeQuaternion();
}


std::ostream& operator << ( std::ostream &os, const QQuaternion &quaternion )
{
	const int size = quaternion.size();

	os << "QQuaternion [";

	for (int i = 0; i < size; i++)
	os << qPrintable(QString("%1").arg(quaternion[i], -8, 'f', 6)) << " ";

	os << "]" << std::endl;
	return os;
	}
