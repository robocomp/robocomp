/*
 *    Copyright (C) ${year} by YOUR NAME HERE
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
#ifndef ${interface_name_upper}_H
#define ${interface_name_upper}_H

// Ice includes
#include <Ice/Ice.h>
#include <${filename_without_extension}.h>

#include <config.h>
#include "genericworker.h"


class ${interface_name}I : public virtual ${module_name}::${interface_name}
{
public:
	${interface_name}I(GenericWorker *_worker);
	~${interface_name}I();

	${interface_methods_definition}

private:

	GenericWorker *worker;

};

#endif
