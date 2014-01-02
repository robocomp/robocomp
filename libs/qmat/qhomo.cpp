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
//Homography class implementation

#include <qmat/qhomo.h>

using namespace RMat;

Homo::Homo(): QMat(3,3)
{
	d=100;
// 	P = QMat::zeros(3,3); // QCamera is 3x3 by default
	Pinv = QMat::zeros(3,3);
	R = QMat::zeros(3,3);
	Tr = QMat::zeros(3,1);
	N = QMat::zeros(3,1);
	H = QMat::zeros(3,3);
};

Homo::Homo(Cam p, QMat r, QMat t, QMat n, T d_): QMat(3,3) , P(p), R(r) , Tr(t), N(n), d(d_)
{
	Pinv = QMat::zeros(3,3);
	Pinv=P.invert();
	H=R-((Tr*N.transpose())&(1./d));
	this->inject(P*H*Pinv,0,0);
};
Homo & Homo::Homo::operator =(QMat A)
{
	Q_ASSERT(0);
//  	if (this != &A)
// 	{
// 		data = A.toVector();
// 		rows = A.nRows();
// 		cols = A.nCols();
// 	}
	return *this;
}

Homo::~Homo()
{
}

void Homo::update(const Cam &p, const QMat &r, const QMat &t, const QMat &n, const T &d_)
{
	P=p;
	R=r;
	Tr=t;
	N=n;
	d=d_;
	Pinv=P.invert();
	H=R-((Tr*(N.transpose()))&(1./d));
	this->inject(P*H*Pinv,0,0);
}
void Homo::setP(Cam p)
{
	P=p; Pinv=P.invert(); H=R-((Tr*(N.transpose()))&(1./d));(* this)=(P*H)*Pinv;
};
void Homo::setPlane(QMat n, T d_)
{
	N=n; d=d_;
	//H=Tr*(N.transpose());
	H(0,0)=Tr(0,0)*N(0,0)/d; H(0,1)=Tr(0,0)*N(1,0)/d;H(0,2)=Tr(0,0)*N(2,0)/d;
	H(1,0)=Tr(1,0)*N(0,0)/d; H(1,1)=Tr(1,0)*N(1,0)/d;H(1,2)=Tr(1,0)*N(2,0)/d;
	H(2,0)=Tr(2,0)*N(0,0)/d; H(2,1)=Tr(2,0)*N(1,0)/d;H(2,2)=Tr(2,0)*N(2,0)/d;

	H(0,0)=R(0,0)-H(0,0); H(0,1)=R(0,1)-H(0,1);H(0,2)=R(0,2)-H(0,2);
	H(1,0)=R(1,0)-H(1,0); H(1,1)=R(1,1)-H(1,1);H(1,2)=R(1,2)-H(1,2);
	H(2,0)=R(2,0)-H(2,0); H(2,1)=R(2,1)-H(2,1);H(2,2)=R(2,2)-H(2,2);

	//H=H&(1./d);
	//H=R-H;
	//H=R-((Tr*(N.transpose()))&(1./d));
	this->inject((P*H)*Pinv,0,0);
};
void Homo::setR(QMat r)
{
	R=r; H=R-((Tr*(N.transpose()))&(1./d));this->inject(P*H*Pinv,0,0);
};
void Homo::setT(QMat t)
{
	Tr=t; H=R-((Tr*(N.transpose()))&(1./d));this->inject(P*H*Pinv,0,0);
};

