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

#include <qmat/qessential.h>

using namespace RMat;

QEssential::QEssential() :  QMat(3,3)
{
}

QEssential::QEssential(const QMat &rot, const QMat &trans) : QMat(3,3)
{
	set(rot, trans);
}

QEssential::QEssential(const QMat &rot, const QVec &trans) : QMat(3,3)
{
	set(rot, trans);
}

QEssential::QEssential(const QEssential &c)
{
}

QEssential::~QEssential()
{
}

void QEssential::set(const QMat &rot, const QMat &trans)
{
// 	rot.print("rot");
// 	trans.print("trans");
	(*this).inject(trans * rot,0,0);
}

void QEssential::set(const QMat &rot, const QVec &trans)
{
	QMat T = trans.crossProductMatrix();
	set(rot, T);
}




