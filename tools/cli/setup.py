from setuptools import setup, find_packages

setup(
    name='robocomp_cli',
    version='0.0.1.2',
    packages=['robocomp','rcbuild', 'rccd', 'rcconfig', 'rcdocker', 'rcportchecker', 'rcrun', 'rcworkspace', 'robocompdsl'],
    package_dir={
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
    # packages_dir=['rcbuild.rcbuild', 'rccd', 'rcconfig', 'rcdocker', 'rcportchecker', 'rcrun', 'rcworkspace', 'robocompdsl'],
    # packages=find_packages()
    # packages=find_packages(include=['rcbuild.rcbuild', 'rccd.rccd', 'rcconfig.rcconfig', 'rcdocker.rcdocker', 'rcportchecker.rcportchecker', 'rcrun.rcrun', 'rcworkspace.rcworkspace', 'robocompdsl.robocompdsl'], exclude=["build", "robocompdsl.*"]),
    # # packages_dir={
    # #     'robocomp': 'robocomp/'
    # # },
    # # install_requires=[
    # #     'robocomp',
    # #     'rcbuild'
    # # ],
    # url='https://github.com/robocomp/robocomp/tree/development/tools/cli',
    # license='GPL V3',
    # author='Esteban Martinena',
    # author_email='emartinena@unex.es',
    # description='Main CLI robocomp application',
    # long_description=open('README.md').read(),
    # long_description_content_type='text/markdown',
    install_requires=[
        'typer', 'docker', 'pyyaml', 'prompt_toolkit'
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
