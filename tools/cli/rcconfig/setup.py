from setuptools import setup, find_packages

setup(
    name='rcconfig',
    version='0.0.1',
    packages=find_packages(),
    url='https://github.com/robocomp/robocomp/tree/development/tools/cli/rcconfig',
    license='GPL V3',
    author='Esteban Martinena',
    author_email='emartinena@unex.es',
    description='Main CLI rcconfig application',
    entry_points={'console_scripts': ['rcconfig = rcconfig.rcconfig:app']}

)
