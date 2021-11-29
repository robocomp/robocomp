from setuptools import setup, find_packages

setup(
    name='robocompcli',
    version='2021.11.02',
    packages=['robocompcli','robocomp','rcbuild', 'rccd', 'rcconfig', 'rcdocker', 'rcportchecker', 'rcrun', 'rcworkspace', 'robocompdsl'],
    package_dir={
	    'robocompcli': '.',
        'robocomp': 'robocomp/robocomp/',
        'rcbuild': 'rcbuild/rcbuild/',
        'rccd': 'rccd/rccd/',
        'rcconfig': 'rcconfig/rcconfig/',
        'rcdocker': 'rcdocker/rcdocker/',
        'rcportchecker': 'rcportchecker/rcportchecker/',
        'rcrun': 'rcrun/rcrun/',
        'rcworkspace': 'rcworkspace/rcworkspace/',
        'robocompdsl': 'robocompdsl/robocompdsl/'
    },
    include_package_data=True,
    package_data={
        # If any package contains *.txt files, include them:
        # And include any *.dat files found in the "data" subdirectory
        # of the "mypkg" package, also:
        "robocompdsl": [
            "common/**",
            "templates/*",
            "templates/**/*",
            "templates/**/**/*",
            "templates/**/**/**/*",
            "templates/**/**/**/**/*",
            "templates/**/**/**/**/**/*",
            "dsl_parsers/*",
            "dsl_parsers/**/*",
            "dsl_parsers/**/**/*",
        ]
    },
    install_requires=[
        'typer', 'docker', 'pyyaml', 'prompt_toolkit', "ruamel.yaml"
    ],
    entry_points={
        'console_scripts': [
            'robocomp = robocomp.robocomp:app',
            'rcbuild = rcbuild.rcbuild:app',
            'rccd = rccd.rccd:app',
            'rcconfig = rcconfig.main:app',
            'rcdocker = rcdocker.rcdocker:app',
            'rcportchecker = rcportchecker.rcportchecker:app',
            'rcrun = rcrun.rcrun:app',
            'rcworkspace = rcworkspace.main:app',
            'robocompdsl = robocompdsl.main:app',
        ]
    }

)
