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
// Camara related matrices class

#ifndef ESSENTIAL_H
#define ESSENTIAL_H

#include <qmat/qmat.h>

//! Camera related matrices derivation of QMat with dedicated methods
/**
	\brief Essential matrix for a stereo rig
	@author Pablo Bustos
*/


namespace RMat
{
	class QEssential : public QMat
	{
		public:
			QEssential();
			QEssential(const QMat &rot, const QVec &trans);
			QEssential(const QMat &rot, const QMat &trans);
			QEssential(const QEssential &c);
			~QEssential();

			void set(const QMat &rot, const QVec &trans);
			void set(const QMat &rot, const QMat &trans);


		private:

	};


};
#endif

