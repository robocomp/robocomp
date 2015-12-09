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
// Camara related matrices class

#ifndef FUNDAMENTAL_H
#define FUNDAMENTAL_H

#include <qmat/qmat.h>
#include <qmat/qessential.h>
#include <qmat/qcamera.h>


//! Fundamental matrix for stero rig 
/**
	\brief Fudamental matrix for stero rig
	@author Pablo Bustos
*/

namespace RMat
{
	
	class QFundamental : public QMat
	{
	public:
		/** Default constructor. **/
		QFundamental();
		/** Copy constructor. **/
		QFundamental( const QFundamental & c);
		/** Parametrized constructor: Given the essential matrix and both camera matrices builds the fundamental matrix. **/
		QFundamental(const QEssential & essential, const Cam &kL, const Cam &kR);
		/** Destructor **/
		~QFundamental();

		/** Resets the fundamental matrix: Given the essential matrix and both camera matrices rebuilds the fundamental matrix. **/
		void set( const QEssential & essential, const Cam & kL, const Cam & kR );
		QLineF getEpipolarR(const QPoint & pI, float x1=0, float x2=320);
		float getEpipolarRheight(const QPoint & pD, float x);
		QLineF getEpipolarL(const QPoint & pD, float x1=0, float x2=320);
		float getEpipolarLheight(const QPoint & pD, float x);
		T getDistToEpipolar(const QPoint & pI, const QPoint & pD);

	private:
		float focalLeft, focalRight;
	};

};


#endif

