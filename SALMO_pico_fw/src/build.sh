#! /bin/bash

rm -r ../build
mkdir ../build
cd ../build
cmake ..
cd src
make -j4
exit
