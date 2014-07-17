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
// 2D and 3D Rotation matrices class

#ifndef QROT3DOX_H
#define QROT3DOX_H

#include <qmat/qmat.h>


//! 2D and 3D Rotation matrices derivation of QMat with dedicated methods
/**
	\brief
	@author Pablo Bustos
*/


namespace RMat
{
	//Rotation 3D
	class Rot3DOnAxis : public QMat
	{
		public:
			Rot3DOnAxis(T alfa=0);
			Rot3DOnAxis( const Rot3DOnAxis &m):QMat(m) {}
			virtual ~Rot3DOnAxis() {}
			virtual void update(T alfa) = 0;
			T getAlfa() const { return ang; }
		protected:
			T ang;
	};
	
	class Rot3DOX : public Rot3DOnAxis
	{
		public:
			Rot3DOX(T alfa=0);
			Rot3DOX( const Rot3DOX &m);
			Rot3DOX( const Rot3DOnAxis &m):Rot3DOnAxis(m) {}
			virtual ~Rot3DOX();
			void update(T alfa);
	};

	//Rotation 3DC
	class Rot3DCOX : public Rot3DOnAxis
	{
		public:
			Rot3DCOX(T alfa=0);
			Rot3DCOX( const Rot3DCOX &m);
			Rot3DCOX( const Rot3DOnAxis &m):Rot3DOnAxis(m) {}
			virtual ~Rot3DCOX();
			void update(T alfa);
	};
	
	class Rot3DOY : public Rot3DOnAxis
	{
		public:
			Rot3DOY(T alfa=0);
			Rot3DOY( const Rot3DOY &m);
			Rot3DOY( const Rot3DOnAxis &m):Rot3DOnAxis(m) {}			
			virtual ~Rot3DOY();
			void update(T alfa);
	};
	//Rotation 3DC
	class Rot3DCOY : public Rot3DOnAxis
	{
		public:
			Rot3DCOY(T alfa=0);
			Rot3DCOY( const Rot3DCOY &m);
			Rot3DCOY( const Rot3DOnAxis &m):Rot3DOnAxis(m) {}			
			virtual ~Rot3DCOY();
			void update(T alfa);
	};
	
	class Rot3DOZ : public Rot3DOnAxis
	{
		public:
			Rot3DOZ(T alfa=0);
			Rot3DOZ( const Rot3DOZ &m);
			Rot3DOZ( const Rot3DOnAxis &m):Rot3DOnAxis(m) {}			
			virtual ~Rot3DOZ();
			void update(T alfa);
	};
	
	class Rot3DCOZ : public Rot3DOnAxis
	{
		public:
			Rot3DCOZ(T alfa=0);
			Rot3DCOZ( const Rot3DCOZ &m);
			Rot3DCOZ( const Rot3DOnAxis &m):Rot3DOnAxis(m) {}			
			virtual ~Rot3DCOZ();
			void update(T alfa);
	};
	
	class Rot3D : public QMat
	{
		public:
			Rot3D(T ox=0, T oy=0, T oz=0, bool XCW=true, bool YCW=true,bool ZCW=true);
			Rot3D(const Rot3D &ex);
			virtual ~Rot3D();
			Rot3D operator= (const Rot3D&  ex);
			void update(T ox, T oy, T oz);

		private:
			bool XC, YC, ZC;
			Rot3DOnAxis * RX;
			Rot3DOnAxis * RY;
			Rot3DOnAxis * RZ;
	};
	class Rot3DC : public QMat
	{
		public:
			Rot3DC(T ox=0, T oy=0, T oz=0, bool XCW=true, bool YCW=true,bool ZCW=true);
			Rot3DC(const Rot3DC &ex);
			virtual ~Rot3DC();
			Rot3DC operator= (const Rot3DC&  ex);
			void update(T ox, T oy, T oz);

		private:
			bool XC, YC, ZC;
			Rot3DOnAxis * RX;
			Rot3DOnAxis * RY;
			Rot3DOnAxis * RZ;
	};
	
	/*! 2D clockwise Rotation matrix
	 \brief Rotates alfa radians clockwise
	*/
	class Rot2D : public QMat
	{
		public:
			Rot2D(T alfa);
			virtual ~Rot2D();
			void update(T alfa);
	};

	/*! 2D counterclockwise Rotation matrix
	 \brief Rotates alfa radians counterclockwise
	*/
	class Rot2DC : public QMat
	{
		public:
			Rot2DC(T alfa);
			virtual ~Rot2DC();
			void update(T alfa);
	};

};
#endif

