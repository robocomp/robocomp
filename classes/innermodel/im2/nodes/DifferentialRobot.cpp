/*
 *    Copyright (C) 2010-2013 by RoboLab - University of Extremadura
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

#include "../innermodel.h"

namespace IM2 {

// ------------------------------------------------------------------------------------------------
// DifferentialRobot
// ------------------------------------------------------------------------------------------------

DifferentialRobot::DifferentialRobot(
	QString id_,
	float tx_,
	float ty_,
	float tz_,
	float rx_,
	float ry_,
	float rz_,
	uint32_t port_,
	Node *parent_)
: Body( id_, RTMat( rx_, ry_, rz_, tx_, ty_, tz_), BodyParams( Kinematic, 0.0f, 0.0f, 0.0f ), parent_ )
{
	im_tx = tx_;
	im_ty = ty_;
	im_tz = tz_;
	im_rx = rx_;
	im_ry = ry_;
	im_rz = rz_;
	im_port = port_;
}



DifferentialRobot::~DifferentialRobot()
{
}



void DifferentialRobot::print(bool verbose)
{
}



void DifferentialRobot::save(QTextStream &out, int tabs)
{
}



void DifferentialRobot::setRotation( const float rx_, const float ry_, const float rz_ )
{
	im_rx = rx_;
	im_ry = ry_;
	im_rz = rz_;
	im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty, im_tz );
}



void DifferentialRobot::setTranslation( const float tx_, const float ty_, const float tz_ )
{
	im_tx = tx_;
	im_ty = ty_;
	im_tz = tz_;
	im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty, im_tz );
}



void DifferentialRobot::setTransform( const float tx_, const float ty_, const float tz_, const float rx_, const float ry_, const float rz_ )
{
	im_rx = rx_;
	im_ry = ry_;
	im_rz = rz_;
	im_tx = tx_;
	im_ty = ty_;
	im_tz = tz_;
	im_pose.set( im_rx, im_ry, im_rz, im_tx, im_ty, im_tz );
}

}
