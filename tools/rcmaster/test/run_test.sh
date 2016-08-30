#!/bin/sh
echo "Please kill any rcmaster instances running before running tests\n"
rcnode &
python -m unittest -v test
pkill icebox

