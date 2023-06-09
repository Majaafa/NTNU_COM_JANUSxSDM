# NTNU_COM_JANUSxSDM 2023
This is continued work from an earlier project: https://github.com/markerv/NTNU_ROV_COM

The SDMSH library used is found on this GitHub: https://github.com/EvoLogics/sdmsh

More details for the JANUS library used can be found on the JANUS wiki: https://www.januswiki.com/tiki-index.php

For this project was VirtualBox used: https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview

(link to algorithm developed in the same project: https://github.com/HaIvor/Optimization)


Details about the code and the project can be found on the GitHub wiki to this repository and in the bachelor report.

## Install

To get started you will need to install the following in the linux terminal:

### Make and cmake
```
make -version #Check if make is installed
sudo apt install make
sudo apt install build-essential
sudo snap install cmake --classic
```

### libreadline
```
sudo apt-get install libreadline-dev
```
## Implementation 

### FFTW3

```
cd lib/fftw/fftw-3.3.10/
./configure
make
sudo make install
make check
```
### Compiling 

For a virtual machine: compile the SDMSH library.
```
cd lib/sdmsh
make
```
For the PI: the SDMSH folder should be deleted and cloned again before compiling.
```
cd lib/
rm -r sdmsh/
git clone https://github.com/evologics/sdmsh.git
cd sdmsh/
make
```
(NB! when deleting the SDMSH, the preamble.raw file will also be deleted. Therefore must this file be added to the SDMSH library again.)

Setup and compile the Janus-c-3.0.5 library:
```
cd lib/janus-c-3.0.5/
cmake -S . -B bin/
cd bin
make
sudo make install
```


