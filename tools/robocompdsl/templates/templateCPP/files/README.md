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
def SPACE(i=0):
	s = ''
	if i>0:
		s = str(i)
	cog.out('<S'+s+'>')

from dsl_parsers.dsl_factory import DSLFactory
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
]]]
[[[end]]]
[[[cog
cog.out('# ' + component.name)
]]]
[[[end]]]
Intro to component here


## Configuration parameters
[[[cog
cog.out('As any other component, *' + component.name + '* needs a configuration file to start. In')
]]]
[[[end]]]
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
[[[cog
cog.out('cd <' + component.name + '\'s path> ')
]]]
[[[end]]]
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
[[[cog
cog.out('bin/'+component.name + ' config')
]]]
[[[end]]]
```
