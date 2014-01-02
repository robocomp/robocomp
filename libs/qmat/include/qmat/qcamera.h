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

#ifndef CAM_H
#define CAM_H

#include <qmat/qmat.h>


//! Camera related matrices derivation of QMat with dedicated methods
/**
	\brief Camera matrix and Projection Matrix (should also go Fundamental matrix and Essential matrix)
	@author Pablo Bustos
*/
namespace RMat
{
	class QVec;
	/**
	* \class Cam
	* @brief Camera matrix. Parámetros intrínsecos
	*/
	class Cam : public QMat
	{
	private:
		RMat::T *focusX;	/// Pointer to reference horizontal focus value easily
		//!< Pointer to reference vertical focus value easily
		RMat::T *focusY;	
		RMat::T *centerX;	//!< Pointer to reference center X value easily
		RMat::T *centerY;	//!< Pointer to reference center Y value easily
		float focalX;		//!< Horizontal focus
		float focalY;		//!< Vertical focus
		float centroX;		//!< Horizontal position of imagen center in pixel coordinates.
		float centroY;		//!< Vertical position of imagen center in pixel coordinates.
		int width;			//!< 
		int height;			//!< 
		int size;			//!< 
	public:
		Cam();
		Cam( const Cam & c);
		Cam(T Fx, T Fy, T Ox, T Oy);
		~Cam();

		QVec getAngles( const QVec & p) const;
		QMat getAngles( const QMat & p) const;     //Deprecated
		QVec getAnglesHomogeneous( const QVec & p) const;
		float getFocal() const;
		float getFocalX() const;
		float getFocalY() const;
		int getHeight() const;
		int getSize() const;
		QVec getRayHomogeneous( const QVec & p) const;
		QVec getRay(const QVec & p) const;
		int getWidth() const;
		QMat polar3DToCamera(const QMat & p) const ;
		QVec project( const QVec & p) const;
		QMat project( const QMat & p) const;       //Deprecated		
		void set( T Fx, T Fy, T Ox, T Oy );
		void setFocal( const int f) const;
		void setFocalX( const int fx);
		void setFocalY( const int fy);
		void setSize(int w, int h);
		QVec toZeroCenter( const QVec &p) const;
		QMat toZeroCenter( const QMat &p) const;   //Deprecated
		QVec toZeroCenterHomogeneous( const QVec &p) const;
	};
};
#endif

