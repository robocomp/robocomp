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
#ifndef LPOLAR_H
#define LPOLAR_H
#include <QtCore>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/**
@author Pablo Bustos
*/
class LPolar{
private:
    int ecc,ang,w,h,s;
    int **TX,***LX;
    int **TY,***LY;	   
    float base;
	struct TDato { int ang; int ecc; };
	QVector<QVector<TDato > > DILP; //Tabla para reconstruccion inversa densa
	
public:
	LPolar();
    LPolar(int _ecc,int _ang, int _w, int _h);
	void initLPolar(int _ecc,int _ang, int _w, int _h);
    ~LPolar();
    void convert(unsigned char *in, unsigned char *out);
    void inverse(unsigned char *in, unsigned char *out);
    void inverseComplete(unsigned char *in, unsigned char *out , int width);
	void initData();
	void initDataCompleto();
	void initDataDILP(int ecc, int ang, int s);
	void convertPromedio(unsigned char *in, unsigned char *out );
	void convertPointLPtoC(int e, int a, int *x, int*y);
};

#endif
