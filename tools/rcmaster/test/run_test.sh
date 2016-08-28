#!/bin/sh
echo "Please kill any rcmster instances running before running tests\n"
python -m unittest -v test
