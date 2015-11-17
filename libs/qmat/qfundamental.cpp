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

#include <qmat/qfundamental.h>

using namespace RMat;

QFundamental::QFundamental() :  QMat ( 3,3 ) 
{ 

}
	
QFundamental::QFundamental(const QEssential & essential, const Cam & kL, const Cam & kR)
{
	
	( *this ).inject(kR.transpose().invert() * ( essential * kL.invert()), 0,0);
	focalLeft = kL.getFocal();
	focalRight = kR.getFocal();
}

RMat::QFundamental::~ QFundamental()
{
}

void RMat::QFundamental::set(const QEssential & essential, const Cam & kL, const Cam & kR)
{
	focalLeft = kL.getFocal();
	focalRight = kR.getFocal();
	( *this ).inject(kR.transpose().invert() * ( essential * kL.invert()), 0,0);
}

QLineF RMat::QFundamental::getEpipolarL(const QPoint & pI, float x1, float x2)
{
	QVec p(3);
	p(0)=pI.x();
	p(1)=pI.y();
	p(2)=1.;
	//p.transpose().print("antes punto");
	//(*this).print("antes fund");
	
	QVec res = p * (*this);

	/** TESTING NEEDED **/
	// Si b!=0: ax + by + cf = 0 donde a,b,c están en res y x,y,focal son puntos cualesquiera de la imagen derecha
	// para x=0: y = -cf/b
	// para x=320: y = (-cf - ax)/b
	// res.print("paramsR");
	if (res(1)!=0)
		return QLineF(x1, ((-res(2)-res(0)*x1)/res(1)), x2, ((-res(2)-res(0)*x2)/res(1)));
	// Si b==0: ax + by + cf = 0 == ax + cf donde a,b,c están en res y x,y,focal son puntos cualesquiera de la imagen derecha
	// para y= 0: x = -cf/a
	// para y= 240: x = -cf/a
	// res.print("paramsR");
	else if (res(0)!=0)
		return QLineF(-res(2)/res(0), 0, -res(2)/res(0), 480);
	// Degenerated case
	qFatal("Degenerated camera!");
	return QLineF(0,0,0,0);
}

float RMat::QFundamental::getEpipolarLheight(const QPoint &pI, float x)
{
	QVec p(3);
	p(0)=pI.x();
	p(1)=pI.y();
	p(2)=1.;
	QVec res = p * (*this);

	if (res(1)!=0)
		return (-res(2)-res(0)*x)/res(1);
	qFatal("Degenerated camera!");
	return 0;
}

QLineF RMat::QFundamental::getEpipolarR(const QPoint & pD, float x1, float x2)
{
	QMat p(3);
	p(0)=pD.x();
	p(1)=pD.y();
	p(2)=1.;
	QMat res = operator*(p);
	return QLineF(x1, (-res(2) - res(0)*x1)/res(1), x2, (-res(2) - res(0)*x2)/res(1));
}

float RMat::QFundamental::getEpipolarRheight(const QPoint & pD, float x)
{
	QMat p(3);
	p(0)=pD.x();
	p(1)=pD.y();
	p(2)=1.;
	QMat res = operator*(p);
	return (-res(2) - res(0)*x)/res(1);
}

T RMat::QFundamental::getDistToEpipolar(const QPoint & pI, const QPoint & pD)
{
	QVec left = QVec::vec3(pI.x(),pI.y(),1.);
	QVec right = QVec::vec3(pD.x(),pD.y(),1.);
	
	T res = fabs(right * (operator*(left)));
	return res;
	
}


