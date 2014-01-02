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
// Column Vector class
#ifndef QMOVINGROBOT_H
#define QMOVINGROBOT_H

#include <qmat/qmat.h>

	/**
	*\brief Class derived from QMat specialized to represent the coordinate transformation between the Robot base and the World.
	* The origen of Robex reference system is placed at the mid point of drive axis, and at floor level (Y coordinate = 0) with Z positive point forwards, X positive pointing to the right 
	* of the robot as seen from above, and Y positive pointing upwards.
	* @author Robolab staff 
	*
	*
	*/
///Matriz RobotToWorld: y = Rx + T ; Nos dice cómo se ve un punto visto desde el robot, en el SR del mundo.
	
using namespace RMat;	
class MovingRobot : public QMat
{
	public:
		MovingRobot();
		MovingRobot(T alfa, const QMat & t);
		MovingRobot(T ox, T oy, T oz, const QMat & t);
		MovingRobot( const MovingRobot & mr);
		MovingRobot( T alfa); 										
		~MovingRobot();
		void init();
		QMat getTr() const;									
		QMat getR() const;
		T getAlfa() const;
		T getAlpha() const;
		QMat getPose() const;
		void setR( const QMat & r);
		void setT( const QMat & t);
		void setRT(T x, T z, T alfa);								
		void updateR( T incAlfa);									
		void updateT( T incT);										
		void updateRT( T incAlfa, T incT);
		QMat robotToWorld( const QMat & x) const;
		T angleTo(const QMat & v) const;							
		QMat worldToRobot( const QMat & x) const;				
		/**
		 * \brief Transforms a 3D point seen in World's reference system to Robot's reference system
		 * @param x 3-vector of 3D point in World reference system
		 * @return 3-vector of 3D point in Robot reference system
		 */
		inline QVec direct( const QVec & x ) const			{ return R.transpose() * ( x - Tr );};
		/**
		 * \brief Transforms a 3D point seen in Robot's reference system to Worlds's reference system
		 * @param x 3-vector of 3D point in Robot reference system
		 * @return 3-vector of 3D point in World reference system
		 */
		inline QVec inverse( const QVec & x ) const			{ return (R*x) + Tr;};
		
	private:
		QMat R;
		QMat Tr;
};
#endif
