from setuptools import setup, find_packages

setup(
    name='rcbuild',
    version='0.0.1',
    include_package_data=True,
    packages=find_packages(),
	install_requires=[
		'python-apt',
		'docker'
	],
    url='https://github.com/robocomp/robocomp/tree/development/tools/cli/rcbuild',
    license='GPL V3',
    author='Esteban Martinena',
    author_email='emartinena@unex.es',
    description='rcbuild command line',
    entry_points={'console_scripts': ['rcbuild = rcbuild.rcbuild:app']}
)
