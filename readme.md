# Building process
    git pull
    git submodule update --init
    cd pico-sdk

Add to your shell profile .bashrc or .zshrc the current directory of pico-sdk folder as env variable
    
    export PICO_SDK_PATH=path_to/pico-sdk
<br>

    git submodule update --init
    cd ..
    cd SALMO_pico_fw
    mkdir build
    cd build 
    cmake ..
    cd src
    make -j4

After this process some binaries files will be created. <br>The suitable one is the `.uf2` file, which is located into `SALMO_pico_fw/build/src`. 
<br>After this do the following steps:
1. Disconnect the board
1. Hold the `BOOTSEL` button
1. Connect the device to your computer

Then drag and drop it to `RPI-RP2` mass storage device. If the device is not showed by your computer.


# :boom: Compilation routine for the algo

    cd fw-dev
    cd tracking-algotithm
    cd algo

:godmode: For real optimization wizards

    gcc sun_tracker.c -O2 -lm -o out 
    ./out

:hatched_chick: For normal people

    gcc sun_tracker.c -lm  
    ./a.out

:zap: Usage with args

    ./out {year} {month} {day} {hour} {minute} {second} {latitude} {longitude}



    