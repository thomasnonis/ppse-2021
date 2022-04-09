#! /bin/bash
cd ../build
cmake ..
cd src
make -j4
exit