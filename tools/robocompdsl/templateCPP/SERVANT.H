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
modulePool = IDSLPool(theIDSLs)
module = modulePool.moduleProviding(theInterface)
if module == None:
	print('Can\'t locate', theIDSLs)
	sys.exit(1)


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
#ifndef 
[[[cog
A()
cog.out(theInterface.upper())
Z()
]]]
[[[end]]]
_H
#define 
[[[cog
A()
cog.out(theInterface.upper())
Z()
]]]
[[[end]]]
_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <
[[[cog
A()
cog.out(module['filename'].split('/')[-1].split('.')[0])
Z()
]]]
[[[end]]]
.h>

#include <config.h>
#include "genericworker.h"

using namespace 
[[[cog
A()
cog.out(module['name'])
Z()
]]]
[[[end]]]
;

class 
[[[cog
A()
cog.out(theInterface)
Z()
]]]
[[[end]]]
I : public QObject , public virtual 
[[[cog
A()
cog.out(module['name'])
Z()
]]]
[[[end]]]
::
[[[cog
A()
cog.out(theInterface)
Z()
]]]
[[[end]]]

{
Q_OBJECT
public:
	
[[[cog
A()
cog.out(theInterface)
Z()
]]]
[[[end]]]
I( GenericWorker *_worker, QObject *parent = 0 );
	~
	[[[cog
A()
cog.out(theInterface)
Z()
]]]
[[[end]]]
I();
	
[[[cog

	for interface in module['interfaces']:
		if interface['name'] == theInterface:
			for mname in interface['methods']:
				method = interface['methods'][mname]
				
				ret = method['return']
				name = method['name']
				
				paramStr = ''
				for p in method['params']:
					ampersand = '&'
					if p['decorator'] == 'out':
						const = ''
					else:
						const = 'const '
						if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
							ampersand = ''
					paramStr += const + p['type'] + ' ' + ampersand + p['name'] + ', '

				cog.outl('<TABHERE>' + ret + ' ' + name + '(' + paramStr + 'const Ice::Current&);')

]]]
[[[end]]]

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
