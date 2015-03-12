/*
[[[cog

import sys
sys.path.append('/opt/robocomp/python')

import cog
def A():
	cog.out('<@@<')
def Z():
	cog.out('>@@>')
def TAB():
	cog.out('<TABHERE>')

from parseCDSL import *
component = CDSLParsing.fromFile(theCDSL)
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)

from parseIDSL import *
pool = IDSLPool(theIDSLs)


]]]
[[[end]]]
 *    Copyright (C) 
[[[cog
A()
import datetime
cog.out(str(datetime.date.today().year))
Z()
]]]
[[[end]]]
 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

[[[cog
ll = []
if 'implements'   in component: ll += component['implements']
if 'subscribesTo' in component: ll += component['subscribesTo']
for imp in ll:
	module = pool.moduleProviding(imp)
	for interface in module['interfaces']:
		if interface['name'] == imp:
			for mname in interface['methods']:
				method = interface['methods'][mname]
				paramStrA = ''
				for p in method['params']:
					# delim
					if paramStrA == '': delim = ''
					else: delim = ', '
					# decorator
					ampersand = '&'
					if p['decorator'] == 'out':
						const = ''
					else:
						const = 'const '
						if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
							ampersand = ''
					# STR
					paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
				cog.outl("<TABHERE>" + method['return'] + ' ' + method['name'] + '(' + paramStrA + ");")

]]]
[[[end]]]

public slots:
	void compute(); 	

private:
};

#endif

