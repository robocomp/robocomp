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
#include <qmat/qmatrot.h>

using namespace RMat;

Rot3DOnAxis::Rot3DOnAxis(T alfa):QMat(3,3)
{
	ang = alfa;
}

//Rot3DOX Implementation
Rot3DOX::Rot3DOX(T alfa):Rot3DOnAxis(alfa)
{
	(*this)(0,0) = 1.; (*this)(0,1) = 0.;         (*this)(0,2) = 0.;
	(*this)(1,0) = 0.; 
	(*this)(2,0) = 0.; 
	update(alfa);
}

Rot3DOX::Rot3DOX(const Rot3DOX & m):Rot3DOnAxis(m)
{
	ang = m.getAlfa();
}

Rot3DOX::~Rot3DOX(){

}

void Rot3DOX::update(T alfa)
{
	(*this)(1,1) = cos(alfa); (*this)(1,2) = -sin(alfa);
	(*this)(2,1) = sin(alfa);(*this)(2,2) = cos(alfa);	
	ang = alfa;
}

//Rot3DOX Implementation
Rot3DCOX::Rot3DCOX(T alfa):Rot3DOnAxis(alfa)
{	(*this)(0,0) = 1.; (*this)(0,1) = 0.;         (*this)(0,2) = 0.;
	(*this)(1,0) = 0.; 
	(*this)(2,0) = 0.; 
	update(alfa);
}

Rot3DCOX::Rot3DCOX(const Rot3DCOX & m):Rot3DOnAxis(m)
{
	ang = m.getAlfa();
}

Rot3DCOX::~Rot3DCOX(){

}

void Rot3DCOX::update(T alfa)
{
	(*this)(1,1) = cos(alfa); (*this)(1,2) = sin(alfa);
	(*this)(2,1) = -sin(alfa);(*this)(2,2) = cos(alfa);
	ang = alfa;
}



//Rot3DOY Implementation
Rot3DOY::Rot3DOY(T alfa):Rot3DOnAxis(alfa)
{
                              (*this)(0,1) = 0.;
	(*this)(1,0) = 0.;         (*this)(1,1) = 1.; (*this)(1,2) = 0.;
                              (*this)(2,1) = 0.;
	update(alfa);
}

Rot3DOY::Rot3DOY(const Rot3DOY & m):Rot3DOnAxis(m)
{	
	ang = m.getAlfa();
}

Rot3DOY::~Rot3DOY(){};

void Rot3DOY::update(T alfa)
{
	(*this)(0,0) = cos(alfa) ;(*this)(0,2) = sin(alfa);
	(*this)(2,0) = -sin(alfa) ;(*this)(2,2) =  cos(alfa);	
	ang = alfa;
}

Rot3DCOY::Rot3DCOY(T alfa):Rot3DOnAxis(alfa)
{
	                       (*this)(0,1) = 0.;
	(*this)(1,0) = 0.;     (*this)(1,1) = 1.;   (*this)(1,2) = 0.;
	                       (*this)(2,1) = 0.;
	update(alfa);
}

Rot3DCOY::Rot3DCOY(const Rot3DCOY & m):Rot3DOnAxis(m)
{	
	ang = m.getAlfa();
}

Rot3DCOY::~Rot3DCOY(){};

void Rot3DCOY::update(T alfa)
{
	(*this)(0,0) = cos(alfa) ;(*this)(0,2) = -sin(alfa);
	(*this)(2,0) = sin(alfa) ;(*this)(2,2) =  cos(alfa);	
	ang = alfa;
}



//Rot3DOZ Implementation
Rot3DOZ::Rot3DOZ(T alfa):Rot3DOnAxis(alfa)
{
	                                        (*this)(0,2) = 0.;
	                                        (*this)(1,2) = 0.;
	(*this)(2,0) = 0.;  (*this)(2,1) = 0.;  (*this)(2,2) = 1.;
	update(alfa);	
}

Rot3DOZ::Rot3DOZ(const Rot3DOZ & m):Rot3DOnAxis(m)
{	
	ang = m.getAlfa();
}

Rot3DOZ::~Rot3DOZ()
{
}


void Rot3DOZ::update(T alfa)
{
	(*this)(0,0) = cos(alfa);  (*this)(0,1) = -sin(alfa);
	(*this)(1,0) = sin(alfa); (*this)(1,1) = cos(alfa);	
	ang = alfa;
}

//Con C

Rot3DCOZ::Rot3DCOZ(T alfa):Rot3DOnAxis(alfa)
{
	                                                     (*this)(0,2) = 0.;
	                                                     (*this)(1,2) = 0.;
	(*this)(2,0) = 0.;         (*this)(2,1) = 0.;        (*this)(2,2) = 1.;
	update(alfa);	
}

Rot3DCOZ::Rot3DCOZ(const Rot3DCOZ & m):Rot3DOnAxis(m)
{	
	ang = m.getAlfa();
}

Rot3DCOZ::~Rot3DCOZ()
{
}


void Rot3DCOZ::update(T alfa)
{
	(*this)(0,0) = cos(alfa);  (*this)(0,1) = sin(alfa);
	(*this)(1,0) = -sin(alfa); (*this)(1,1) = cos(alfa);
	ang = alfa;

}

//Rot3D Implementation
Rot3D::Rot3D(T ox, T oy, T oz, bool XCW, bool YCW, bool ZCW):QMat(3,3)
{
	XC = XCW;
	YC = YCW;
	ZC = ZCW;
	if(XCW)
		RX = new Rot3DOX(ox);
	else
		RX = new Rot3DCOX(ox);	  
	if(YCW)
		RY = new Rot3DOY(oy);
	else
		RY = new Rot3DCOY(oy);	  
	if(ZCW)
		RZ = new Rot3DOZ(oz);
	else
		RZ = new Rot3DCOZ(oz);	  
	
	this->inject((*RX)*(*RY)*(*RZ),0,0);
}


Rot3D::Rot3D(const Rot3D &ex) : QMat( ex ) 
{
	XC = ex.XC;
	YC = ex.YC;
	ZC = ex.ZC;
	if(XC)
		RX = new Rot3DOX(0);
	else
		RX = new Rot3DCOX(0);
	if(YC)
		RY = new Rot3DOY(0);
	else
		RY = new Rot3DCOY(0);
	if(ZC)
		RZ = new Rot3DOZ(0);
	else
		RZ = new Rot3DCOZ(0);
	
	this->inject((*RX)*(*RY)*(*RZ),0,0);

}

Rot3D::~Rot3D()
{
	delete RX;
	delete RY;
	delete RZ;
}

Rot3D Rot3D::operator= (const Rot3D&  ex)
{
	XC = ex.XC;
	YC = ex.YC;
	ZC = ex.ZC;
	delete RX;
	delete RY;
	delete RZ;
	if(XC)
		RX = new Rot3DOX(0);
	else
		RX = new Rot3DCOX(0);
	if(YC)
		RY = new Rot3DOY(0);
	else
		RY = new Rot3DCOY(0);
	if(ZC)
		RZ = new Rot3DOZ(0);
	else
		RZ = new Rot3DCOZ(0);
	
	this->inject((*RX)*(*RY)*(*RZ),0,0);
	
	return *this;
}

void Rot3D::update(T ox, T oy, T oz)
{
	RX->update(ox);
	RY->update(oy);
	RZ->update(oz);
	this->inject((*RX)*(*RY)*(*RZ),0,0);
}



Rot3DC::Rot3DC(T ox, T oy, T oz, bool XCW, bool YCW, bool ZCW):QMat(3,3)
{
	if(XCW)
		RX = new Rot3DOX(ox);
	else
		RX = new Rot3DCOX(ox);	  
	if(YCW)
		RY = new Rot3DOY(oy);
	else
		RY = new Rot3DCOY(oy);	  
	if(ZCW)
		RZ = new Rot3DOZ(oz);
	else
		RZ = new Rot3DCOZ(oz);	  
	this->inject((*RZ)*(*RY)*(*RX),0,0);
}

Rot3DC::Rot3DC(const Rot3DC &ex) : QMat( ex ) 
{
	XC = ex.XC;
	YC = ex.YC;
	ZC = ex.ZC;
	if(XC)
		RX = new Rot3DOX(0);
	else
		RX = new Rot3DCOX(0);
	if(YC)
		RY = new Rot3DOY(0);
	else
		RY = new Rot3DCOY(0);
	if(ZC)
		RZ = new Rot3DOZ(0);
	else
		RZ = new Rot3DCOZ(0);
	
	this->inject((*RZ)*(*RY)*(*RX),0,0);

}


Rot3DC::~Rot3DC()
{
	delete RX;
	delete RY;
	delete RZ;

}

Rot3DC Rot3DC::operator= (const Rot3DC&  ex)
{
	XC = ex.XC;
	YC = ex.YC;
	ZC = ex.ZC;
	delete RX;
	delete RY;
	delete RZ;

	if(XC)
		RX = new Rot3DOX(0);
	else
		RX = new Rot3DCOX(0);
	if(YC)
		RY = new Rot3DOY(0);
	else
		RY = new Rot3DCOY(0);
	if(ZC)
		RZ = new Rot3DOZ(0);
	else
		RZ = new Rot3DCOZ(0);
	
	this->inject((*RZ)*(*RY)*(*RX),0,0);
	
	return *this;
}


void Rot3DC::update(T ox, T oy, T oz)
{
	RX->update(ox);
	RY->update(oy);
	RZ->update(oz);
	this->inject((*RZ)*(*RY)*(*RX),0,0);
}

//Rot2D Rotates alfa rads clockwise
/**
 * \brief 2D rotation matrix. Rotates alfa rads clockwise
 * @param alfa, angle in radians
 */
Rot2D::Rot2D(T alfa):QMat(2,2)
{
	(*this)(0,0) = cos(alfa) ;	(*this)(0,1) = sin(alfa) ;
	(*this)(1,0) = -sin(alfa);	(*this)(1,1) =   cos(alfa);
}

Rot2D::~Rot2D()
{
}

void Rot2D::update(T alfa)
{	(*this)(0,0) = cos(alfa) ;	(*this)(0,1) = sin(alfa) ;
	(*this)(1,0) = -sin(alfa);	(*this)(1,1) = cos(alfa);
}

/**
 * \brief 2D rotation matrix. Rotates alfa rads counterclockwise
 * @param alfa, angle in radians
 */
Rot2DC::Rot2DC( T alfa ):QMat(2,2)
{
	(*this)(0,0) = cos(alfa) ;	(*this)(0,1) = -sin(alfa) ;
	(*this)(1,0) = sin(alfa);	(*this)(1,1) =   cos(alfa);
}

Rot2DC::~Rot2DC()
{
}


