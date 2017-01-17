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

#ifdef COMPILE_IPP
#include <qmat/ippWrapper.h>
#endif


#include <qmat/qrtmat.h>

using namespace RMat;

RTMat::RTMat(bool XCW, bool YCW, bool ZCW) : QMat ( 4,4 )
{
	XC = XCW;
	YC = YCW;
	ZC = ZCW;
	R = QMat::identity(3);
	if(XCW)
		Rx = new Rot3DOX(0);
	else
		Rx = new Rot3DCOX(0);
	if(YCW)
		Ry = new Rot3DOY(0);
	else
		Ry = new Rot3DCOY(0);
	if(ZCW)
		Rz = new Rot3DOZ(0);
	else
		Rz = new Rot3DCOZ(0);

	Tr = QVec(3);
	Tr.set(0.);
	do_inject();
}

RTMat::RTMat(const RTMat & ex, bool XCW, bool YCW, bool ZCW) : QMat( ex )
{
	XC = XCW;
	YC = YCW;
	ZC = ZCW;
	if(XCW)
		Rx = new Rot3DOX(0);
	else
		Rx = new Rot3DCOX(0);
	if(YCW)
		Ry = new Rot3DOY(0);
	else
		Ry = new Rot3DCOY(0);
	if(ZCW)
		Rz = new Rot3DOZ(0);
	else
		Rz = new Rot3DCOZ(0);

	*Rx = ex.getRx();
	*Ry = ex.getRy();
	*Rz = ex.getRz();
	R = ex.getR();

	Tr = ex.getTr();
	do_inject();
}

RTMat& RMat::RTMat::operator= (const RTMat&  ex)
{

	XC = ex.XC;
	YC = ex.YC;
	ZC = ex.ZC;

	delete Rx;
	delete Ry;
	delete Rz;

	if(XC)
		Rx = new Rot3DOX(ex.getRxValue());
	else
		Rx = new Rot3DCOX(ex.getRxValue());
	if(YC)
		Ry = new Rot3DOY(ex.getRyValue());
	else
		Ry = new Rot3DCOY(ex.getRyValue());
	if(ZC)
		Rz = new Rot3DOZ(ex.getRzValue());
	else
		Rz = new Rot3DCOZ(ex.getRzValue());

	R = ex.getR();

	Tr = ex.getTr();

	do_inject();
	return *this;
}

RTMat::RTMat(const RTMat &ex) : QMat( ex )
{
	XC = ex.XC;
	YC = ex.YC;
	ZC = ex.ZC;
	if(XC)
		Rx = new Rot3DOX(0);
	else
		Rx = new Rot3DCOX(0);
	if(YC)
		Ry = new Rot3DOY(0);
	else
		Ry = new Rot3DCOY(0);
	if(ZC)
		Rz = new Rot3DOZ(0);
	else
		Rz = new Rot3DCOZ(0);

	*Rx = ex.getRx();
	*Ry = ex.getRy();
	*Rz = ex.getRz();
	R = ex.getR();

	Tr = ex.getTr();
	do_inject();
}


RTMat::RTMat(const QMat &ex) : QMat( ex )
{

	Rx = NULL;
	Ry = NULL;
	Rz = NULL;

}


RTMat::RTMat ( T rx, T ry, T rz, const QVec & t, bool XCW, bool YCW, bool ZCW ) : QMat ( 4,4 )
{
	XC = XCW;
	YC = YCW;
	ZC = ZCW;
	if(XCW)
		Rx = new Rot3DOX(rx);
	else
		Rx = new Rot3DCOX(rx);
	if(YCW)
		Ry = new Rot3DOY(ry);
	else
		Ry = new Rot3DCOY(ry);
	if(ZCW)
		Rz = new Rot3DOZ(rz);
	else
		Rz = new Rot3DCOZ(rz);

	R = (*Rx)*(*Ry)*(*Rz);
	Tr = t;
	do_inject();
}

RMat::RTMat::RTMat(T rx, T ry, T rz, T tx, T ty, T tz, bool XCW, bool YCW, bool ZCW): QMat ( 4,4 )
{
	XC = XCW;
	YC = YCW;
	ZC = ZCW;
	if(XCW)
		Rx = new Rot3DOX(rx);
	else
		Rx = new Rot3DCOX(rx);
	if(YCW)
		Ry = new Rot3DOY(ry);
	else
		Ry = new Rot3DCOY(ry);
	if(ZCW)
		Rz = new Rot3DOZ(rz);
	else
		Rz = new Rot3DCOZ(rz);

	R = (*Rx)*(*Ry)*(*Rz);
	Tr = QVec::vec3(tx, ty, tz);
	do_inject();
}


RTMat::~RTMat() {
	if(Rx!=NULL)
	  delete Rx;
	if(Ry!=NULL)
	  delete Ry;
	if(Rz!=NULL)
	  delete Rz;
}

void RMat::RTMat::do_inject()
{
	(*this).inject(R, 0, 0 );
	(*this).inject(Tr, 0, 3 );
	operator()(3,0) = 0.f;
	operator()(3,1) = 0.f;
	operator()(3,2) = 0.f;
	operator()(3,3) = 1.f;
}

RTMat RMat::RTMat::operator *(const RTMat & A) const
{
	Q_ASSERT(cols == A.nRows());
	RTMat C;
#ifdef COMPILE_IPP
	ippmMul_mm_32f(toDataConst(), cols*sizeof(T), sizeof(T), cols, rows, A.toDataConst(), A.nCols()*sizeof(T), sizeof(T), A.nCols(), A.nRows(), C.toData(), C.nCols()*sizeof(T), sizeof(T));
#else
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<A.cols;j++)
		{
			C(i,j)=0;
			for(int k=0;k<cols;k++)
			{
				C(i,j) += operator()(i,k)*A(k,j);
			}
// 			printf("%f ", C(i,j));
		}
// 		printf("\n");
	}
#endif
 	QMat rot(3,3);
	QVec t(3);
 	for (int i=0; i<3; i++)
 		for (int j=0; j<3; j++)
 			rot(i,j) = C(i,j);

	for (int j=0; j<3; j++)
	    t(j) = C(j,3);

	C.setR(rot);
	C.setTr(t);
// 	QVec angles = rot.extractAnglesR();
// 	qDebug()<<"angles"<<angles(0)<< angles(1)<< angles(2);
// 	C.set(angles(0), angles(1), angles(2), C(0, 3), C(1, 3), C(2, 3));
// 	C.print("C Post");
	return C;
}

QVec RMat::RTMat::operator *(const QVec & vector) const
{
	Q_ASSERT(cols == vector.size());
	QVec result ( vector.size() );
#ifdef COMPILE_IPP
	ippmMul_mm_32f(toDataConst(), cols*sizeof(T), sizeof(T), cols, rows, vector.getReadData(), sizeof(T), sizeof(T), 1, vector.size(), result.getWriteData(), sizeof(T), sizeof(T));
#else
	for(int i=0;i<rows;i++)
	{
		result(i)=0;
		for(int k=0;k<cols;k++)
		{
			result(i) += operator()(i,k)*vector(k);
		}
// 		printf("%f ", result(i));
// 		printf("\n");
	}
#endif

	return result;
}


void RMat::RTMat::set(T ox, T oy, T oz, T x, T y, T z)
{
	Tr(0)= x;
	Tr(1)= y;
	Tr(2)= z;

	Rx->update( ox );
	Ry->update( oy );
	Rz->update( oz );
	R = (*Rx)*(*Ry)*(*Rz);

	do_inject();
}


void RMat::RTMat::init(T ox, T oy, T oz, const QVec & t)
{
	Tr = t;
	Rx->update( ox );
	Ry->update( oy );
	Rz->update( oz );
	R = (*Rx)*(*Ry)*(*Rz);

	do_inject();
}

void RTMat::setR( const QMat & rot)
{
	R = rot;
	do_inject();
}

void RTMat::setR ( T ox, T oy, T oz)
{
	Rx->update( ox );
	Ry->update( oy );
	Rz->update( oz );
	R = (*Rx)*(*Ry)*(*Rz);
	do_inject();
}

void RTMat::setTr( const QVec & t )
{
	Tr=t;
	do_inject();
}

void RTMat::setRT ( T ox, T oy, T oz, const QVec & t )
{
	Rx->update( ox );
	Ry->update( oy );
	Rz->update( oz );
	R = (*Rx)*(*Ry)*(*Rz);
	Tr=t;
	do_inject();

}


void RTMat::setRX ( T ox )
{
	Rx->update( ox );
	R = (*Rx)*(*Ry)*(*Rz);
	do_inject();
}

void RTMat::setRY ( T oy )
{
	Ry->update( oy );
	R = (*Rx)*(*Ry)*(*Rx);
	do_inject();
}

void RTMat::setRZ ( T oz )
{
	Rz->update( oz );
	R = (*Rx)*(*Ry)*(*Rz);
	do_inject();
}

QVec RMat::RTMat::getTr() const
{
	return Tr;
}

Rot3DOX RMat::RTMat::getRx() const
{
	return (Rot3DOX)*Rx;
}

Rot3DOY RMat::RTMat::getRy() const
{
	return *Ry;
}

Rot3DOZ RMat::RTMat::getRz() const
{
	return *Rz;
}

QMat RMat::RTMat::getR() const
{
	return 	R;
}

RTMat RMat::RTMat::invert() const
{
	RTMat r;
 	r.R = R.transpose();
 	r.Tr = r.R * (Tr*-1);
	r.inject( r.R , 0 , 0 );
 	r.inject( r.Tr, 0 , 3);

// 	r.inject( R.transpose() , 0 , 0 );
// 	r.inject( R.transpose() * (Tr*-1) , 0 , 3);

 	r(3,3) = 1.f;
	return r;
}

QMat RMat::RTMat::invertR()
{
	QMat r(4,4);
	r.inject( R.transpose() , 0 , 0 );
	r(3,3) = 1.f;
	return r;
}


///RMatC

RTMatC::RTMatC() : QMat ( 4,4 )
{
	R = QMat::identity(3);
	Rx = Rot3DCOX(0);
	Ry = Rot3DCOY(0);
	Rz = Rot3DCOZ(0);
	Tr = QVec(3);
	Tr.set(0.);
	do_inject();
}

RTMatC::RTMatC(const RTMatC & ex) : QMat( ex )
{
	Rx = ex.getRx();
	Ry = ex.getRy();
	Rz = ex.getRz();
	R = Rz*Ry*Rx;
	Tr = ex.getTr();
	do_inject();
}

RTMatC::RTMatC ( T ox, T oy, T oz, const QVec & t ) : QMat ( 4,4 )
{
	Rx = Rot3DCOX( ox );
	Ry = Rot3DCOY( oy );
	Rz = Rot3DCOZ( oz );
	R = Rz*Ry*Rx;
	Tr = t;
	do_inject();
}

RMat::RTMatC::RTMatC(T ox, T oy, T oz, T x, T y, T z): QMat ( 4,4 )
{
	Rx = Rot3DCOX( ox);
	Ry = Rot3DCOY( oy);
	Rz = Rot3DCOZ( oz);
	R = Rz*Ry*Rx;
	Tr.resize(3);
	Tr(0)= x;
	Tr(1)= y;
	Tr(2)= z;
	do_inject();
}

RTMatC::~RTMatC() {
}

void RMat::RTMatC::do_inject()
{
	(*this).inject(R, 0, 0 );
	(*this).inject(Tr, 0, 3 );
	operator()(3,0) = 0.f;
	operator()(3,1) = 0.f;
	operator()(3,2) = 0.f;
	operator()(3,3) = 1.f;
}

RTMatC RMat::RTMatC::operator *(const RTMatC & A) const
{
	Q_ASSERT(cols == A.nRows());
	RTMatC C;
#ifdef COMPILE_IPP
	ippmMul_mm_32f(toDataConst(), cols*sizeof(T), sizeof(T), cols, rows, A.toDataConst(), A.nCols()*sizeof(T), sizeof(T), A.nCols(), A.nRows(), C.toData(), C.nCols()*sizeof(T), sizeof(T));
#else
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<A.nCols();j++)
		{
			C(i,j) = 0;
			for(int k=0;k<cols;k++)
			{
				C(i,j) += operator()(i,k) * A(k,j);
			}
		}
	}
#endif
	return C;
}

QVec RMat::RTMatC::operator *(const QVec & vector) const
{
	Q_ASSERT(cols == vector.size());
	QVec result ( vector.size() );
#ifdef COMPILE_IPP
	ippmMul_mm_32f(toDataConst(), cols*sizeof(T), sizeof(T), cols, rows, vector.getReadData(), sizeof(T), sizeof(T), 1, vector.size(), result.getWriteData(), sizeof(T), sizeof(T));
#else
	for(int i=0;i<rows;i++)
	{
		result(i) = 0;
		for(int k=0;k<cols;k++)
		   result(i) += operator()(i,k) * vector(k);
	}
#endif

	return result;
}


void RMat::RTMatC::set(T ox, T oy, T oz, T x, T y, T z)
{
	Tr(0)= x;
	Tr(1)= y;
	Tr(2)= z;

	Rx = Rot3DCOX( ox );
	Ry = Rot3DCOY( oy );
	Rz = Rot3DCOZ( oz );
	R = Rz*Ry*Rx;


	do_inject();
}


void RMat::RTMatC::init(T ox, T oy, T oz, const QVec & t)
{
	Tr = t;
	Rx = Rot3DCOX( ox );
	Ry = Rot3DCOY( oy );
	Rz = Rot3DCOZ( oz );
	R = Rz*Ry*Rx;

	do_inject();
}

void RTMatC::setR ( T ox, T oy, T oz)
{
	Rx = Rot3DCOX( ox );
	Ry = Rot3DCOY( oy );
	Rz = Rot3DCOZ( oz );
	R = Rz*Ry*Rx;
;
	( *this ).inject ( R , 0 , 0 );
}

void RTMatC::setTr( const QVec & t )
{
	Tr=t;
	( *this ).inject ( Tr, 0, 3 );
}

void RTMatC::setRT ( T ox, T oy, T oz, const QVec & t )
{
	Rx = Rot3DCOX( ox );
	Ry = Rot3DCOY( oy );
	Rz = Rot3DCOZ( oz );
	R = Rz*Ry*Rx;
	Tr=t;
	( *this ).inject ( R , 0, 0 );
	( *this ).inject ( Tr, 0, 3 );
}

void RTMatC::setRX ( T ox )
{
	Rx = Rot3DCOX( ox );
	R = Rz*Ry*Rx;
	( *this ).inject ( R , 0 , 0 );
}

void RTMatC::setRY ( T oy )
{
	Ry = Rot3DCOY( oy );
	R = Rz*Ry*Rx;
	( *this ).inject ( R , 0 , 0 );
}

void RTMatC::setRZ ( T oz )
{
	Rz = Rot3DCOZ( oz );
	R = Rz*Ry*Rx;
	( *this ).inject ( R , 0 , 0 );
}

QVec RMat::RTMatC::getTr() const
{
	return Tr;
}

Rot3DCOX RMat::RTMatC::getRx() const
{
	return Rx;
}

Rot3DCOY RMat::RTMatC::getRy() const
{
	return Ry;
}

Rot3DCOZ RMat::RTMatC::getRz() const
{
	return Rz;
}

QMat RMat::RTMatC::getR() const
{
	   return Rz*Ry*Rx;
}

RTMatC RMat::RTMatC::invert() const
{
	RTMatC r;
 	r.R = R.transpose();
 	r.Tr = r.R * (Tr*-1);
	r.inject( r.R , 0 , 0 );
 	r.inject( r.Tr, 0 , 3);
	r(3,3) = 1.f;
	return r;
}

QMat RMat::RTMatC::invertR()
{
	QMat r(4,4);
	r.inject( R.transpose() , 0 , 0 );
	r(3,3) = 1.f;
	return r;
}



///////////////////////////////
//// Transformations
//////////////////////////////

/**
 * \brief Transforms from world towards robot reference frames: y = R' * (p - Tr)
 * @param p
 * @return coordinates of p as seen from next (towards robot tip) reference frame
 */
// QVec RTMat::direct ( const QVec &p ) const
// {
// 	return R.transpose() * ( p - Tr );
// }

/**
 * \brief Transforms from robot to world reference frames: y = R*p + Tr
 * @param p
 * @return coordinates of p as seen from previous (towards world) reference frame
 */
// QVec RMat::RTMat::inverse ( const QVec & p ) const
// {
// 	return R*p + Tr;
// }

/**
 * \brief Transforms (only translation) from world towards robot reference frames: y = p - Tr)
 * @param p
 * @return coordinates of p as seen from next (towards robot tip) reference frame
 */

// QVec RMat::RTMat::directTr ( const QVec & p ) const
// {
// 	return p - Tr;
// }
//
// /**
//  * \brief Transforms (only translation) from robot to world reference frames: y = p + Tr
//  * @param p
//  * @return coordinates of p as seen from previous (towards world) reference frame
//  */
// QVec RMat::RTMat::inverseTr ( const QVec & p ) const
// {
// 	return p + Tr;
// }

