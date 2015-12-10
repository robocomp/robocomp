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

ExtendedRangeSensor::ExtendedRangeSensor(const RoboCompLaser::TLaserData &laserData, const RoboCompDifferentialRobot::TBaseState &bState, InnerModel *innerModel_, double extensionRange_, double maxDist_, QString laserName_)
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
	double window[5];
	double x[laserData->size()];

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
			const double temp = window[j];
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
int ExtendedRangeSensor::angleToExtendedIndex( double angle)
{
	while (angle>M_PI)  angle -= 2.*M_PI;
	while (angle<-M_PI) angle += 2.*M_PI;

	double ret;
// 	angle += extensionRange/2.;
	ret = (angle * dataExtended.size())/extensionRange;
	ret += dataExtended.size()/2;
	return int32_t(ret);
}


void ExtendedRangeSensor::update(const RoboCompLaser::TLaserData &laserData)
{
	QVector<TExt> dataExtendedBack = dataExtended;

	/// a) Inicialización
	for (int i=0; i<dataExtended.size(); i++)
	{
		setExtended(i, maxDist, true);
	}

	/// b) Transformación a t+1
	for (int i=0; i<dataExtendedBack.size(); i++)
	{
		QVec l = innerModel->transform(laserName, dataExtendedBack[i].world, "root");
		double sens = l.norm2();
		double angle = atan2(l(0), l(2));
		printf("%g -> %g\n", dataExtendedBack[i].angle, angle);
		int index = angleToExtendedIndex(angle);
		if (fabs(angle) > extensionRange/2. and sens < maxDist)
		{
// 			printf ("%g (%g)-> %d %d\n", double(180.*angle/M_PIl), double(sens), i, index);
// 			dataExtendedBack[i].world.print("w");
		}
		if (sens < maxDist)
		{
			if (index >= 0 and index < dataExtended.size())
			{
				setExtended(index, sens, true);
			}
		}
	}

	/// c) copiamos datos leídos sobre datos extendidos
	for (unsigned int i=0; i<laserData.size(); i++)
	{
		const int32_t index = angleToExtendedIndex(laserData[i].angle);
		double sens = laserData[i].dist;
		if (sens > maxDist)
			sens = maxDist;
		setExtended(index, sens, true);
	}

	/// d) interpolación de los puntos no visitados
// 	interpolation();


	/// Make a copy
	for(int h=0; h<dataExtended.size(); h++)
	{
		laserDataExtCopy[h].dist = dataExtended[h].dist;
		laserDataExtCopy[h].angle = dataExtended[h].angle;
	}
// 	medianFilter(&laserDataExtCopy);
}


void ExtendedRangeSensor::setExtended(int i, double dist, bool visit, double certainty)
{
	if (i<0 or i>=dataExtended.size()) qFatal("%s %d: i<0 or i>=dataExtended.size()\n", __FILE__, __LINE__);
	dataExtended[i].dist = dist;
	QVec v = innerModel->laserTo("root", laserName, dist, dataExtended[i].angle);
	dataExtended[i].world = v;
	dataExtended[i].visit = visit;
	dataExtended[i].certainty = certainty;
}

void ExtendedRangeSensor::interpolation(int first, int last)
{
	Q_ASSERT(first<last);
	Q_ASSERT(first > 0 and first < dataExtended.size());
	Q_ASSERT(last  > 0 and  last < dataExtended.size());

	for (int i=first; i<=last; i++)
	{
		if (dataExtended[i].visit != true or dataExtended[i].dist > maxDist-1)
		{
			printf("notvistied %d\n", i);
			for (int o = 1; o<2; o++)
			{
				if (i+o < dataExtended.size())
				{
					if (dataExtended[i+o].visit == true and dataExtended[i+o].dist < maxDist)
					{
						setExtended(i, dataExtended[i+o].dist, false);
						break;
					}
				}
				if (i-o > -1)
				{
					if (dataExtended[i-o].visit == true and dataExtended[i-o].dist < maxDist)
					{
						setExtended(i, dataExtended[i-o].dist, false);
						break;
					}
				}
			}
		}
	}
}


// void ExtendedRangeSensor::relax(const double quantity, InnerModel *im, const QString &platformRef, const QString &worldRef)
// {
// 	for (uint32_t i=0; i<size(); i++)
// 	{
// 		dataExtended[i].relax(quantity, im, platformRef, worldRef);
// // 		laserDataExtCopy[i].dist += q;
// 	}
// }

