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
//QExtrinsics cpp file

#include <qmat/qextrinsics.h>

using namespace RMat;

QExtrinsics::QExtrinsics() : QMat ( 3,4 ) , R(QMat::identity(3)) , Tr(QMat(3,1))
{
	( *this ).inject ( R, 0, 0 );
	Tr.set(0.);
	( *this ).inject ( Tr, 0, 3 );
};

QExtrinsics::QExtrinsics(const QExtrinsics & ex) : QMat( ex ) ,   Rx( ex.getRx()), Ry( ex.getRy()), R( ex.getR() ) , Tr( ex.getTr() )
{
	( *this ).inject ( R, 0, 0 );
	( *this ).inject ( Tr, 0, 3 );
}

QExtrinsics::QExtrinsics ( T ox, T oy, const QMat & t ) : QMat ( 3,4 ) , Rx( Rot3DOX( ox )) , Ry( Rot3DOY( oy )) , R( Rx*Ry ), Tr( t )
{
	( *this ).inject ( R, 0, 0 );
	( *this ).inject ( Tr, 0, 3 );
};

RMat::QExtrinsics::QExtrinsics(T ox, T oy, T x, T y, T z): QMat ( 3,4 ) , Rx( Rot3DOX( ox )) , Ry( Rot3DOY( oy )) , R( Rx*Ry ), Tr( 3 )
{
	Tr(0)= x;
	Tr(1)= y;
	Tr(2)= z;

	( *this ).inject ( R, 0, 0 );
	( *this ).inject ( Tr, 0, 3 );

}

QExtrinsics::~QExtrinsics()
{};

void RMat::QExtrinsics::set(T ox, T oy, T x, T y, T z)
{
	Tr(0)= x;
	Tr(1)= y;
	Tr(2)= z;
	
	Rx = Rot3DOX( ox );
	Ry = Rot3DOY( oy ); 
	R = ( Rx*Ry );
	( *this ).inject ( R, 0, 0 );
	( *this ).inject ( Tr, 0, 3 );
}


void RMat::QExtrinsics::init(T ox, T oy, const QMat & t)
{
	Tr = t;
	Rx = Rot3DOX( ox );
	Ry = Rot3DOY( oy ); 
	R = ( Rx*Ry );
	( *this ).inject ( R, 0, 0 );
	( *this ).inject ( Tr, 0, 3 );
}

void QExtrinsics::setR ( T ox, T oy )
{
	Rx = Rot3DOX( ox );
	Ry = Rot3DOY( oy ); 
	R = Rx * Ry;
	( *this ).inject ( R , 0 , 0 );
};

void QExtrinsics::setTr( const QMat & t )
{
	Tr=t;
	( *this ).inject ( Tr, 0, 3 );
};

void QExtrinsics::setRT ( T ox, T oy, const QMat & t )
{
	Rx = Rot3DOX( ox );
	Ry = Rot3DOY( oy );
	R = Rx * Ry;
	Tr=t;
	( *this ).inject ( R , 0, 0 );
	( *this ).inject ( Tr, 0, 3 );
};

void QExtrinsics::setRX ( T ox )
{
	Rx = Rot3DOX( ox );
	R = Rx * Ry;
	( *this ).inject ( R , 0 , 0 );
};

void QExtrinsics::setRY ( T oy )
{
	Ry = Rot3DOY( oy );
	R = Rx * Ry;
	( *this ).inject ( R , 0 , 0 );
};

QMat RMat::QExtrinsics::getTr() const
{
	return Tr;
}

Rot3DOX RMat::QExtrinsics::getRx() const
{
	return Rx;
}

Rot3DOY  RMat::QExtrinsics::getRy() const
{
	return Ry;
}

QMat RMat::QExtrinsics::getR() const
{
	return Rx*Ry;
}

/////////DEPRECATED

QMat QExtrinsics::baseToCamera ( const QMat &p ) const
{
	return R.transpose() * ( p - Tr );
}

QMat RMat::QExtrinsics::cameraToBase ( const QMat & p ) const
{
	return R*p + Tr;
}

//////////////////////////////////////////////////////

QMat QExtrinsics::direct ( const QMat &p ) const
{
	return R.transpose() * ( p - Tr );
}

QMat RMat::QExtrinsics::inverse ( const QMat & p ) const
{
	return R*p + Tr;
}

QVec QExtrinsics::direct ( const QVec &p ) const
{
	return R.transpose() * ( p - Tr.toVector() );
}

QVec RMat::QExtrinsics::inverse ( const QVec & p ) const
{
	return R*p + Tr;
}

QVec RMat::QExtrinsics::directTr ( const QVec & p ) const
{
	return p - Tr;
}

QVec RMat::QExtrinsics::inverseTr ( const QVec & p ) const
{
	return p + Tr;
}


