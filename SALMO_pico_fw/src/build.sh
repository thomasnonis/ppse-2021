#! /bin/bash

if [ "$1" == "all" ];
then
	echo "Emptying build folder..."
	rm -r ../build
	mkdir ../build
fi
mkdir ../build
cd ../build
cmake ..
cd src
make -j4
exit
