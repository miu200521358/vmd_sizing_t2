@echo off
cls

cd /d %~dp0

python setup_clear.py

cd src && python setup.py clean && python setup.py build_ext --force --inplace && cd ..

