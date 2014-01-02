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
// MovingRobot

#include <qmat/qmovingrobot.h>
#include <qmat/qvec.h>
#include <qmat/qmatrot.h>

using namespace RMat;

MovingRobot::MovingRobot() : QMat ( 3,4,0. ) , R ( QMat::identity ( 3 ) ), Tr ( QMat ( 3,1,0. ) )
{
	this->inject ( R , 0,0 );
	this->inject ( Tr,0,3 );
};
MovingRobot::MovingRobot ( T alfa, const QMat & t ) : QMat ( 3,4,0. ) , R ( Rot3DOY ( alfa ) ) , Tr ( t )
{
	this->inject ( R, 0,0 );
	this->inject ( Tr ,0,3 );
};
MovingRobot::MovingRobot ( T ox, T oy, T oz, const QMat & t ) : QMat ( 3,4,0. ) , R ( Rot3DOX ( ox ) * ( Rot3DOY ( oy ) *Rot3DOZ ( oz ) ) ) , Tr ( t )
{
	this->inject ( R, 0,0 );
	this->inject ( Tr ,0,3 );
};

MovingRobot::MovingRobot(T alfa) : QMat(3,4,0.) , R ( Rot3DOY( alfa) ) ,Tr(3)
{
	Tr.set( 0. );
	this->inject ( R, 0,0 );
	this->inject ( Tr ,0,3 );
}

MovingRobot::MovingRobot(const MovingRobot & mr) : QMat(3,4,0.) , R( mr.getR()) , Tr( mr.getTr())
{
	this->inject ( mr.getR(), 0,0 );
	this->inject ( mr.getTr() ,0,3 );
}

MovingRobot::~MovingRobot() {};

void MovingRobot::init()
{
	R.makeIdentity(); Tr=0.;
	this->inject ( R , 0,0 );
	this->inject ( Tr,0,3 );
};

void MovingRobot::setR ( const QMat & r )
{
	R=r;
	this->inject ( R, 0,0 );
};

void MovingRobot::setT ( const QMat & t )
{
	Tr=t;
	this->inject ( Tr ,0,3 );
};

void MovingRobot::setRT ( T x, T z, T alfa )
{
	R=Rot3DOY ( alfa );
	Tr ( 0,0 ) =x; Tr ( 1,0 ) =0.; Tr ( 2,0 ) =z;
	this->inject ( R , 0,0 );
	this->inject ( Tr,0,3 );
}

//Incrementos de R en el sistema de ref del robot. El incremento de ángulo dado por el odómetro
void MovingRobot::updateR ( T incAlfa )
{
	Rot3DOY giro ( incAlfa );
	R = R*giro;
};

//Incrementos de T en el sistema de ref del robot. El incremento de posición dado por el odómetro
void MovingRobot::updateT ( T incT )
{
	QMat dt ( 3,1,0. );
	dt ( 2,0 ) = incT;
	Tr += dt;
};

//Actualizamos R y T; Giramos la acumulada Tr para ponerla en el SR del robot; sumanos a Tr el avance actual;
void MovingRobot::updateRT ( T incAlfa, T incT )
{
	Rot3DOY giro ( incAlfa );
	R = R*giro;
	QVec dt ( 3,0. );
	dt ( 2 ) = incT;
	Tr += R*dt;
};

QMat MovingRobot::getTr() const
{
	return ( Tr );
};


QMat MovingRobot::getPose() const
{
	QMat pose(3);
	pose(0) = Tr(0);
	pose(1) = Tr(2);
	pose(2) = getAlfa();
	return pose;
}

QMat MovingRobot::getR() const
{
	return R;
}

//Devuelve el vector que une al SR del mundo con el robot en coord del SRMundo
T MovingRobot::getAlpha() const
{
	return -atan2 ( R ( 2,0 ),R ( 0,0 ) );
};

//Devuelve el vector que une al SR del mundo con el robot en coord del SRMundo
T MovingRobot::getAlfa() const
{
	return -atan2 ( R ( 2,0 ),R ( 0,0 ) );
};

//Transforma un punto visto del mundo  al SR del robot
QMat MovingRobot::robotToWorld ( const QMat & x ) const
{
	return (R*x) + Tr;

}

T MovingRobot::angleTo ( const QMat & v ) const
{
	return atan2 ( v ( 0,0 ),v ( 2,0 ) );
};

//Transforma un punto visto desde el robot al SR del mundo
QMat MovingRobot::worldToRobot( const QMat & x ) const
{
	return R.transpose() * ( x - Tr );
}




