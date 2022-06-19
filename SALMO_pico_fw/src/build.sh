#! /bin/bash

if [ "$1" == "all" ];
then
	echo "Removing build folder..."
	rm -r ../build
	mkdir ../build
fi
cd ../build
cmake ..
cd src
make -j4
exit
