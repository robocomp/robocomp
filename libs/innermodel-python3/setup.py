import sys
sys.path.append ('.')

import setuptools
from distutils.core import setup

with open ("README", "r") as fh:
    long_description = fh.read()

setup(
    name="innermodel_python3",
    version="0.0.1",
    author="Akash Kumar Singh",
    author_email="akashsingh049811@gmail.com",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ksakash/innermodel-python3",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)

