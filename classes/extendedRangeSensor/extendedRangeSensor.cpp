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
#include "extendedRangeSensor.h"

ExtendedRangeSensor::ExtendedRangeSensor(const RoboCompLaser::TLaserData &laserData, const RoboCompDifferentialRobot::TBaseState &bState, InnerModel *innerModel_, float extensionRange_, float maxDist_, QString laserName_)
{
	laserName = laserName_;
	innerModel = innerModel_;
	extensionRange = extensionRange_;
	maxDist = maxDist_;
	printf("ExtendedRangeSensor angle:%f  dist:%f\n", extensionRange, maxDist);

	LECTURAS = laserData.size();
	TAM_DATAEXT = rint( extensionRange * laserData.size() / ((double)fabs(laserData[0].angle-laserData[laserData.size()-1].angle)));
	pm = TAM_DATAEXT/2;
	dataExtended.resize(TAM_DATAEXT);
	laserDataExtCopy.resize(TAM_DATAEXT);

	for (int i=0; i<dataExtended.size(); i++)
	{
		dataExtended[i].angle = extIndToRads(i);
		dataExtended[i].dist = maxDist;
		setExtended(i, maxDist, true, 1);
	}
	printf("Extended field of view < %f -- %f >\n", dataExtended[0].angle, dataExtended[TAM_DATAEXT-1].angle);
}


void ExtendedRangeSensor::medianFilter(RoboCompLaser::TLaserData *laserData)
{
	float window[5];
	float x[laserData->size()];

	for (int32_t i=2; i<(int)laserData->size()-2; ++i)
	{
		for (int j=0; j<5; ++j)
		{
			window[j] = laserData->at(i - 2 + j).dist;
		}
		int j;
		for (j=0; j<3; ++j)
		{
			int min = j;
			for (int k = j + 1; k < 5; ++k)
				if (window[k] < window[min])
					min = k;
			const float temp = window[j];
			window[j] = window[min];
			window[min] = temp;
		}
		if (window[j] >= maxDist)
		{
			if (window[j-1] >= maxDist)
			{
				x[i] = window[j-2];
			}
			else
			{
				x[i] = window[j-1];
			}
		}
		else
		{
			x[i] = window[j];
		}
	}
	for (int32_t i=0; i<(int)laserData->size(); i++)
	{
		setExtended(angleToExtendedIndex(laserData->at(i).angle), x[i], true);
	}

	// Endings
	setExtended(angleToExtendedIndex(laserData->at(dataExtended.size()-1-1).angle), laserData->at(dataExtended.size()-1-1-1).dist, true);
	setExtended(angleToExtendedIndex(laserData->at(dataExtended.size()-1).angle), laserData->at(dataExtended.size()-1-1).dist, true);
	setExtended(angleToExtendedIndex(laserData->at(1).angle), laserData->at(1+1).dist, true);
	setExtended(angleToExtendedIndex(laserData->at(0).angle), laserData->at(1).dist, true);
}

/**
 * Converts Laser Index to Extended Index
 * @param ind laser index
 * @return extended index
 */
int ExtendedRangeSensor::laserIndToExtInd (int ind)
{
	int r = ind + TAM_DATAEXT/2 - LECTURAS/2;
	return  r;
}

/**
 * Converts extended index to radians
 * @param ind
 * @return
 */
double ExtendedRangeSensor::extIndToRads ( int ind )
{
	const double rads = ((double)ind - dataExtended.size()/2) * (extensionRange / dataExtended.size() );
	return rads;
}

/**
 * Converts angle in radians to extended index
 * @param angle
 * @return
 */
int ExtendedRangeSensor::angleToExtendedIndex( float angle)
{
	while (angle>M_PI)  angle -= 2.*M_PI;
	while (angle<-M_PI) angle += 2.*M_PI;

	float ret;
	ret = (angle * dataExtended.size())/extensionRange;
	ret = ret + dataExtended.size()/2.;
	return int32_t(ret);
}


void ExtendedRangeSensor::update(const RoboCompLaser::TLaserData &laserData)
{
	QVector<TExt> dataExtendedBack = dataExtended;

	/// a) Inicialización
	for (int i=0; i<dataExtended.size(); i++)
	{
		setExtended(i, maxDist, false);
	}

	/// b) Transformación a t+1
	QVec l;
	for (int i=0; i<dataExtended.size(); i++)
	{
		l = innerModel->transform(laserName, dataExtendedBack[i].world, "root");
		float sens = l.norm2();
		if (sens < maxDist)
			setExtended(angleToExtendedIndex(atan2(l(0), l(2))), sens, true);
	}

	/// c) copiamos datos leídos sobre datos extendidos
	for (unsigned int i=0; i<laserData.size(); i++)
	{
		const int32_t index = angleToExtendedIndex(laserData[i].angle);
		float sens = laserData[i].dist;
		if (sens > maxDist)
			sens = maxDist;
		setExtended(index, sens, true);
	}

	/// d) interpolación de los puntos no visitados
// 	interpolation();

	/// e) copy edges
// 	dataExtended[0] = dataExtended[1];
// 	dataExtended[dataExtended.size()-1] = dataExtended[dataExtended.size()-2];

	/// Make a copy
	for(int h=0; h<dataExtended.size(); h++)
	{
		laserDataExtCopy[h].dist = dataExtended[h].dist;
		laserDataExtCopy[h].angle = dataExtended[h].angle;
	}
// 	medianFilter(&laserDataExtCopy);
}


void ExtendedRangeSensor::setExtended(int i, float dist, bool visit, float certainty)
{
	if (i<0 or i>=dataExtended.size()) return;
	dataExtended[i].dist = dist;
	QVec v = innerModel->laserTo(laserName, "root", dist, dataExtended[i].angle);
	dataExtended[i].world = v;
	dataExtended[i].visit = visit;
	dataExtended[i].certainty = certainty;
}

void ExtendedRangeSensor::interpolation(int first, int last)
{
	if (first == -1 or first < 0) first = 0;
	if (last  == -1 or last >= dataExtended.size()) last = dataExtended.size()-1;

	int firstVisited = dataExtended.size();
	int lastVisited = -1;

	for (int i=first; i<=last; i++)
	{
		if (dataExtended[i].visit == true)
		{
			if (i < firstVisited) firstVisited = i;
			if (i > lastVisited)  lastVisited =  i;
		}
	}

	int lastGap = last - lastVisited;
	Q_ASSERT(lastGap >=0);
	int firstGap = firstVisited - first;
	Q_ASSERT(firstGap >=0);
	int borderGap = firstGap + lastGap;
	Q_ASSERT(firstVisited < dataExtended.size() and lastVisited < dataExtended.size());

	/// Tope circular
	if (first==0 and last==dataExtended.size()-1)
	{
		if (firstGap != 0)
		{
			setExtended(0, ((double)firstGap*dataExtended[lastVisited].dist + (double)lastGap*dataExtended[firstVisited].dist) / (double)(borderGap), true);
		}
		if (lastGap != 0)
		{
			setExtended(dataExtended.size()-1, ((double)firstGap*dataExtended[lastVisited].dist + (double)lastGap*dataExtended[firstVisited].dist) / (double)(borderGap), true);
		}
	}
	/// Tope no circular
	else
	{
		if (firstGap != 0) first += firstGap;
		if (lastGap != 0) last -= lastGap;
	}

	Q_ASSERT(first>=0 and first < dataExtended.size()-1 and last>=0 and last < dataExtended.size() and first <= last);

	int le,ri;
	for (int i=first+1; i<=last; i++)
	{
		if (dataExtended[i].visit == false)
		{
			for (le=i+1; le<last; ++le) if (dataExtended[le].visit == true) break; //towards left
			for (ri=i-1; ri>=first; --ri) if (dataExtended[ri].visit == true) break; //towards right
			uint gap = abs(ri-le);
			if (gap <= 26)
			{
				setExtended(i, ((double)(le-i)*dataExtended[ri].dist + (double)(i-ri)*dataExtended[le].dist)/(double)((i-ri)+(le-i)), false);
			}
		}
	}
}


void ExtendedRangeSensor::relax(const float quantity, InnerModel *im, const QString &platformRef, const QString &worldRef)
{
	for (uint32_t i=0; i<size(); i++)
	{
		dataExtended[i].relax(quantity, im, platformRef, worldRef);
// 		laserDataExtCopy[i].dist += q;
	}
}

