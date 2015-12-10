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
#ifndef EXTENDEDRANGESENSOR_H
#define EXTENDEDRANGESENSOR_H

#include <DifferentialRobot.h>
#include <Laser.h>

#include <qmat/QMatAll>
#include <innermodel/innermodel.h>

class TExt
{
public:
	TExt()
	{
		dist = 0;
		angle = 0;
		visit = 0;
		certainty = 0;
		world = QVec::vec3();
	}
	TExt(const TExt& m2)
	{
		dist = m2.dist;
		angle = m2.angle;
		visit = m2.visit;
		certainty = m2.certainty;
		world = m2.world;
	}
	TExt &operator = (const TExt& m2)
	{
		dist = m2.dist;
		angle = m2.angle;
		visit = m2.visit;
		certainty = m2.certainty;
		world = m2.world;
		return *this;
	}
// 	void relax(const double quantity, InnerModel *im, const QString &platformRef, const QString &worldRef)
// 	{
// 		dist += quantity;
// 		QVec l = im->transform(platformRef, world, worldRef);
// 		double newNorm = l.norm2() + quantity;
// 		l = l.normalize();
// 		world = im->transform(worldRef, l.operator*(newNorm), platformRef);
// 	}
public:
	double dist;
	double angle;
	QVec world;
	bool visit;
	double certainty;
};

class ExtendedRangeSensor
{
public:
	///
	ExtendedRangeSensor(const RoboCompLaser::TLaserData &laserData, const RoboCompDifferentialRobot::TBaseState &bState, InnerModel *innerModel_, double extensionRange_, double maxDist_, QString laserName);
	///
	void update(const RoboCompLaser::TLaserData &laserData);

	///
	RoboCompLaser::TLaserData getData() { return laserDataExtCopy; }
	inline RoboCompLaser::TData getData(int32_t i) { return laserDataExtCopy[i]; }
	inline double getRange(uint i) { return dataExtended[i].dist; }
	inline QMat getWorld(uint i) { return dataExtended[i].world; }
	inline uint size() { return dataExtended.size(); }

// 	void relax(const double quantity, InnerModel *im, const QString &platformRef, const QString &worldRef);


private:
	void setExtended(int index, double dist, bool visit=false, double certainty=1.);
	void interpolation(int first=-1, int last=-1);

	QString laserName;
	int LECTURAS;
	int TAM_DATAEXT;
	double extensionRange, maxDist;
	QVector<TExt> dataExtended;
	RoboCompLaser::TLaserData laserDataExtCopy;

	RoboCompDifferentialRobot::TBaseState bState;
int32_t pm;
	InnerModel *innerModel;

	void medianFilter(RoboCompLaser::TLaserData *laserData);
	int laserIndToExtInd(int ind);
	double extIndToRads(int ind);
	int angleToExtendedIndex(double angle);


};


#endif
