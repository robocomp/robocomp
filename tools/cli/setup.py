from setuptools import setup, find_packages, find_packages

setup(
    name='robocomp',
    version='0.0.1',
    # packages=['rcbuild', 'rccd', 'rcconfig', 'rcdocker', 'rcportchecker', 'rcrun', 'rcworkspace', 'robocompdsl'],
    # packages=find_packages(),
    packages=find_packages(include=['rcbuild.rcbuild', 'rccd.rccd', 'rcconfig.rcconfig', 'rcdocker.rcdocker', 'rcportchecker.rcportchecker', 'rcrun.rcrun', 'rcworkspace.rcworkspace', 'robocompdsl.robocompdsl'], exclude=["build", "robocompdsl.*"]),
    url='https://github.com/robocomp/robocomp/tree/development/tools/cli',
    license='GPL V3',
    author='Esteban Martinena',
    author_email='emartinena@unex.es',
    description='Main CLI robocomp application',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    # entry_points={
    #     'console_scripts': [
    #         'rcbuild = rcbuild.rcbuild.rcbuild:app',
    #         'rccd = rccd.rccd.rccd:app'
    #         'rcconfig = rcconfig.rcconfig.rcconfig:app',
    #         'rcdocker = rcdocker.rcdocker.rcdocker:app',
    #         'rcportchecker = rcportchecker.rcportchecker.rcportchecker:app'
    #         'rcrun = rcrun.rcrun.rcrun:app',
    #         'rcworkspace = rcworkspace.rcworkspace.rcworkspace:app',
    #         'robocompdsl = robocompdsl.robocompdsl.main:app',
    #     ]
    # }

)
