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
#include "lpolar.h"

LPolar::LPolar()
{
}

/**
 * Constructor
 * @param _ecc Number of eccentricities
 * @param _ang Number of angular divisions
 * @param _w Number of columns in cartesian image
 * @param _ecc Number of rows in cartesian image
**/

LPolar::LPolar(int _ecc, int _ang, int _w, int _h) 
{
	initLPolar(_ecc , _ang , _w , _h);
}

LPolar::~LPolar()
{

}

/**
 * Initialization of conversion tables
 * @param _ecc eccentricities
 * @param _ang angular divisions
 * @param _w width of cartesian image
 * @param _h height of cartesian image
 */
void LPolar::initLPolar(int _ecc, int _ang, int _w, int _h)
{
	//Class variables	
	ecc=_ecc;ang=_ang;w=_w;h=_h;
	//Force a square image	
	if (w < h) s = w;
	else s= h;

	//Tables for short conversion
	TX = new int*[ecc];
	for(int i=0;i<ecc;i++)
  		TX[i] = new int[ang];
	TY = new int*[ecc];
	for(int i=0;i<ecc;i++)
  		TY[i] = new int[ang];

	//Tables for conversion using the mean of region
	LX = new int**[ecc];
	for(int i=0;i<ecc;i++)
		LX[i] = new int*[ang];
	LY = new int**[ecc];
	for(int i=0;i<ecc;i++)
  		LY[i] = new int*[ang];

	//Initialize 2D array of TData for dense inverse
	for(int i=0; i<s; i++)
	{
		QVector<TDato> cols(s);
		DILP.push_back(cols);
	}

	initData();
	initDataCompleto();
	initDataDILP(ecc,ang,s);
}
	
void LPolar::convert(unsigned char *in, unsigned char *out )
{
	int i,j;
		
	for (i=1;i<ecc;i++)
  	  for (j=0;j<ang;j++){
 			 *(out+(i*ang+j))= in[TY[i][j]*s+TX[i][j]];
	}
}
void LPolar::convertPromedio(unsigned char *in, unsigned char *out )
{
	int i,j,k,tam,sum;
	
	for (i=1;i<ecc;i++)
  	  for (j=0;j<ang;j++){
  	  	tam=LX[i][j][0];
  	  	sum=0;
  	  	for(k=1;k<tam+1;k++)
  	  		sum += in[LY[i][j][k]*s+LX[i][j][k]];
 			 	*(out+(i*ang+j))= sum/tam;
 			}
}
void LPolar::inverse(unsigned char *in, unsigned char *out )
{
	int i,j;
	
	for (i=1;i<ecc;i++)
  	  for (j=0;j<ang;j++){
			out[TY[i][j]*s+TX[i][j]]= in[i*ang+j];
	}
}
// Inverse conversion with interpolation to fill the whole cartesian image
void LPolar::inverseComplete(unsigned char *in, unsigned char *out, int width )
{
	int i,j;
	
	width=width;
	for (i=0;i<s;i++)
  	  for (j=0;j<s;j++)
		{
 			*(out++)= in[DILP[j][i].ecc*ang+DILP[j][i].ang];
		}
}
void LPolar::initData( )
{
	float p,alfa,base;
	float radius=s/2.;
	int x,y;
  	
	base = exp(log(radius)/((float)ecc));  //Such that pow(base,ecc) = radius
	for (int i=1;i<ecc;i++)
    	for (int j=0;j<ang;j++)
			{
      			p = pow(base,i);
     		  	alfa = (float)j*(2.*M_PI)/(float)ang - M_PI;  //From -PI to PI
     		  	x = (int)round(p*cos(alfa)+radius+1);
        		y = (int)round(p*sin(alfa)+radius+1);
        		if (x >= s) x=s-1;
        		if (y >= s) y=s-1;
        		TX[i][j]=x; 
        		TY[i][j]=y;
   			}

}

// Inicializaci�n completa de tablas

void LPolar::initDataCompleto( )
{
  float p,alfa;
  int x,y,tamx=0,tamy=0,k,l,tam,radio,xL,yL;
  
	if (w < h) s = w; 
	else s= h; 
	
	base = exp(log((float)s/2.)/((float)ecc));
	for (int i=1;i<ecc;i++){
		p = pow(base,i);	
	  	radio = (int)round((M_PI / (float)ang ) * p);
	//  	radio += radio * 0.25;
    	for (int j=0;j<ang;j++){
      		alfa = (float)j*(2.*M_PI)/(float)ang - M_PI;
        	x = (int)round(p*cos(alfa)+(s/2)+1);
        	y = (int)round(p*sin(alfa)+(s/2)+1);
        	if (x >= s) x=x-1;
        	if (y >= s) y=y-1;
        	tam=0; 
        	//calculo cuantos elementos hay
        	for(k=x-radio;k<x+radio;k++)
        		for(l=y-radio;l<y+radio;l++)
        		   if(sqrt(((k-x)*(k-x))+((l-y)*(l-y))) <= radio) {
         			 tam++;
        			}
        	//hago sitio y relleno
        	if(tam==0){
        		LX[i][j]=new int[2];
        		LX[i][j][0]=1;
        		LX[i][j][1]=x;
        		LY[i][j]=new int[2];
        		LY[i][j][0]=1;
        		LY[i][j][1]=y;
        	}
        	else{
        		tamx=1;
        		tamy=1;
        		LX[i][j]=new int[tam+1];
        		LX[i][j][0]=tam;
        		LY[i][j]=new int[tam+1];
        		LY[i][j][0]=tam;
        		for(k=x-radio;k<x+radio;k++)
        			for(l=y-radio;l<y+radio;l++)
        		   		if(sqrt(((k-x)*(k-x))+((l-y)*(l-y))) <= radio)
						{
						  	xL=k;
						  	yL=l;
        		    		if(xL>(s-1)) xL=(s-1);
        		    		if(yL>(s-1)) yL=(s-1); 
         			  		if(xL<0) xL=0;
         					if(yL<0) yL=0;
         			  		LX[i][j][tamx++]=xL;
         			  		LY[i][j][tamy++]=yL;
        				}
        	}
         // printf("n campo %d",tam);
        }
   }
}

/**
 * Initialize Dense Inverse Log Polar Table
 * Fill each DILP[x][y] element with N <ecc,ang,dist> values so a
 * weighted sum can be computed for reconstrucion
 */
void LPolar::initDataDILP(int ecc, int ang, int s)
{
	float base,mod,lmod,alfa,lalfa,x,y;
	float radius= (float)s/2.;
	  		
	base = exp(log(radius)/((float)ecc));  //Such that pow(base,ecc) = radius

	for(int i=0; i<s; i++)
		for(int j=0; j<s; j++)
		{
			x = i - radius;
			y = j - radius;
			//Compute mod
			mod = sqrt(x*x +y*y);
			if(mod  >= radius) mod = radius-1;
			//Compute log-mod from base using log-base conversion. Result in 0..ecc range as computed in base
			lmod = log(mod)/log(base);
			//compute angle from atan2. Result in -Pi and Pi range
			alfa = atan2(y,x);
			//take alfa to 0..ang range
			lalfa = alfa*(float)ang/(2.* M_PI) + (ang/2);
			//closest ecc to lmod is (int)lmod.
			DILP[i][j].ecc = (int)rintf(lmod);
			DILP[i][j].ang = (int)rintf(lalfa);
			
		}
		
}

//CUIDADO. USANDO LAS TABLAS TX,TY
/**
 * @param e eccentricties
 * @param a angular divisions
 * @param x x image coordinate
 * @param y y image coordinate
 */
void LPolar::convertPointLPtoC( int e, int a, int * x, int * y )
{
  *x = TX[e][a];
  *y = TY[e][a];
}


