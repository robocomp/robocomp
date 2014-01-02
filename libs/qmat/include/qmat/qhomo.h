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
// Homography matrix class

#ifndef QHOMO_H
#define QHOMO_H

#include <qmat/qmat.h>
#include <qmat/qcamera.h>


//! Homography matriz derivation of QMat with dedicated methods
/**
	\brief
	@author Pablo Bustos
*/


namespace RMat
{
	class Homo : public RMat::QMat
	{
	public:
		public:
		Homo();
		Homo(Cam p, QMat r, QMat t, QMat n, T d_);
		Homo & operator=(QMat A);
		~Homo();
		void update(const Cam &p, const QMat &r, const QMat &t, const QMat &n, const T &d_);
		void setP(Cam p);
		void setPlane(QMat n, T d_);
		void setR(QMat r);
		void setT(QMat t);
	private:
		Cam P;
		QMat Pinv;
		QMat R;
		QMat Tr;
		QMat N;
		QMat H;
		T d;
	};
};
#endif

