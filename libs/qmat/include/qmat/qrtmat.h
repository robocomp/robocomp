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
#ifndef QRTMAT_H
#define QRTMAT_H

#include <qmat/QMatAll>

/**
	\brief Basic 4x4 projective (rotation+translation) matrix to be used in modelling kinematic links
	* Inherits from generic matrix class QMat and adds specific methods to initialize, update and transform points en 3D space.
	@author Robolab
*/
// class InnerModel;
// namespace InnerModel2 {
// 	class InnerModel;
// }

namespace RMat
{
	class RTMat : public QMat
	{
// 	friend class InnerModel;
// 	friend class InnerModel2::InnerModel;
	public:
		RTMat(bool XCW=true, bool YCW=true,bool ZCW=true);
		RTMat( const RTMat & ex, bool XCW, bool YCW, bool ZCW);
		RTMat( T rx, T ry, T rz, const QVec &t, bool XCW=true, bool YCW=true,bool ZCW=true);
		RTMat( T rx, T ry, T rz, T tx, T ty, T tz, bool XCW=true, bool YCW=true,bool ZCW=true);
		~RTMat();
		RTMat(const QMat &ex);
		RTMat(const RTMat &ex);
		RTMat operator==(const RTMat &ex)
		{
			*this = RTMat(ex);
			return *this;
		}
		bool XC, YC, ZC;

		RTMat& operator= (const RTMat&  ex);
		RTMat  operator* ( const RTMat & A ) const ;
		QVec  operator* ( const QVec & v ) const ;
//		RTMat  operator* ( const QMat & A ) const {};
		void init( T ox, T oy, T oz, const QVec & t);
		void set(T ox, T oy, T oz, T x, T y, T z);
		void setR( const QMat & rot);
		void setR( T ox, T oy, T oz);
		void setRX( T ox );
		void setRY( T oy );
		void setRZ( T oz );
		void setTr( T x, T y, T z) { setTr(QVec::vec3(x, y, z)); }
		void setTr( const QVec & t) ;
		void setRT( T ox, T oy, T oz, const QVec & t );
		QVec getTr() const;
		QMat getR() const;
		Rot3DOX getRx() const;
		T getRxValue() const { return Rx->getAlfa(); }
		Rot3DOY getRy() const;
		T getRyValue() const { return Ry->getAlfa(); }
		Rot3DOZ getRz() const;
		T getRzValue() const { return Rz->getAlfa(); }
		RTMat invert() const;
		QMat invertR();

// 		QVec direct ( const QVec &p ) const;
// 		QVec inverse ( const QVec & p ) const;
// 		QVec directTr ( const QVec &p ) const;
// 		QVec inverseTr ( const QVec & p ) const;

		void do_inject();

		Rot3DOnAxis * Rx;
		Rot3DOnAxis * Ry;
		Rot3DOnAxis * Rz;
		QMat R;
		QVec Tr;
	};

	//not clockwise
	class RTMatC : public QMat
	{
// 	friend class InnerModel;
// 	friend class InnerModel2::InnerModel;
	public:
		RTMatC();
		RTMatC( const RTMatC & ex);
		RTMatC( T ox, T oy, T oz, const QVec &t);
		RTMatC( T ox, T oy, T oz, T x, T y, T z);
		~RTMatC();
		RTMatC  operator* ( const RTMatC & A ) const ;
		QVec  operator* ( const QVec & v ) const ;
//		RTMatC  operator* ( const QMat & A ) const {};
		void init( T ox, T oy, T oz, const QVec & t);
		void set(T ox, T oy, T oz, T x, T y, T z);
		void setR( T ox, T oy, T oz);
		void setRX( T ox );
		void setRY( T oy );
		void setRZ( T oz );
		void setTr( const QVec & t) ;
		void setRT( T ox, T oy, T oz, const QVec & t );
		QVec getTr() const;
		QMat getR() const;
		Rot3DCOX getRx() const;
		T getRxValue() const { return Rx.getAlfa(); }
		Rot3DCOY getRy() const;
		T getRyValue() const { return Ry.getAlfa(); }
		Rot3DCOZ getRz() const;
		T getRzValue() const { return Rz.getAlfa(); }
		RTMatC invert() const;
		QMat invertR();

// 		QVec direct ( const QVec &p ) const;
// 		QVec inverse ( const QVec & p ) const;
// 		QVec directTr ( const QVec &p ) const;
// 		QVec inverseTr ( const QVec & p ) const;

		void do_inject();

		Rot3DCOX Rx;
		Rot3DCOY Ry;
		Rot3DCOZ Rz;
		QMat R;
		QVec Tr;
	};

};

#endif
