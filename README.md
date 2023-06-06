# NTNU_COM_JANUSxSDM 2023
This is continued work from an earlier project: https://github.com/markerv/NTNU_ROV_COM

For this project was VirtualBox used: https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview


## Install

To get started you will need to install the following in the linux terminal:

### Make and cmake
```
make -version #Check if make is installed
sudo apt install make
sudo apt install build-essential
sudo snap install cmake --classic
```

### FFTW3

```
wget http://fftw.org/fftw-3.3.10.tar.gz
tar -xzf fftw-3.3.10.tar.gz
cd fftw-3.3.10
./configure
make
sudo make install
make check
```

### Install libreadline
```
sudo apt-get install libreadline-dev
```


## Compiling

### SDMSH
"$project/lib/":
```
make
```

### JANUS
"$project/lib/janus-c-3.0.5/":
```
cmake -S . -B bin/
cd bin
make .
sudo make install
```

## Modem setup
This step must be done for each modem, every time they are turned on:
```
nc ModemIP 9200
+++ATP
```

## Raspberry Pi setup

## Run code with ROS
For the code to run in ros.....
```

```
## Problems with the code
## TIPS :)
