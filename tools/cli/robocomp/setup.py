from setuptools import setup, find_packages

setup(
    name='robocomp',
    version='0.0.1',
    packages=find_packages(),
    url='https://github.com/robocomp/robocomp/tree/development/tools/robocomp',
    license='GPL V3',
    author='Esteban Martinena',
    author_email='emartinena@unex.es',
    description='Main CLI robocomp application',
    entry_points={'console_scripts': ['robocomp = robocomp.robocomp:app']}

)
