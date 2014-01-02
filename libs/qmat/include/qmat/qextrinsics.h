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
#ifndef QEXTRINSICS_H
#define QEXTRINSICS_H

#include <qmat/QMatAll>


//! Camera extrinsics parameters matrix derived from QMat with dedicated methods
/**
	\brief Camera matrix and Projection Matrix (should also go Fundamental matrix and Essential matrix)
	@author Pablo Bustos
*/

namespace RMat
{
	class QExtrinsics : public QMat
	{
	public:
		QExtrinsics();
		QExtrinsics( const QExtrinsics & ex);
		QExtrinsics( T ox, T oy, const QMat &t);
		QExtrinsics( T ox, T oy, T x, T y, T z);
		~QExtrinsics();
		void init( T ox, T oy, const QMat & t);
		void set(T ox, T oy, T x, T y, T z);
		void setR( T ox, T oy);
		void setRX( T ox );
		void setRY( T oy );
		void setTr( const QMat & t) ;
		void setRT( T ox, T oy, const QMat & t );
		QMat getTr() const;
		QMat getR() const;
		Rot3DOX getRx() const;
		T getRxValue() const { return Rx.getAlfa(); }
		Rot3DOY getRy() const;
		T getRyValue() const { return Ry.getAlfa(); }
		QMat baseToCamera( const QMat &p) const; 	//!< Takes a point seen from base reference system and returns it seen from the camera ref. system
		QMat cameraToBase( const QMat &p) const;	//!< Takes a point seen from the camera ref. system and returns it seen from the base ref. system.
		QMat direct ( const QMat &p ) const;	//!< Same as baseToCamera
		QMat inverse ( const QMat & p ) const;//!< Same as cameraToBase
		QVec direct ( const QVec &p ) const;	//!< Same as baseToCamera
		QVec inverse ( const QVec & p ) const;//!< Same as cameraToBase
		QVec directTr ( const QVec &p ) const;	//!< Same as baseToCamera
		QVec inverseTr ( const QVec & p ) const;//!< Same as cameraToBase

	
	private:
		Rot3DOX  Rx;
		Rot3DOY  Ry;
		QMat R, Tr;
	};

};

#endif
