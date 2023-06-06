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
### compiling 

For a virtual machine: compile the SDMSH library.
```
cd lib/smdmsh
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

Setup and compile the Janus-c-3.0.5 library:
```
cd lib/janus-c-3.0.5/
cmake -S . -B bin/
cd bin
make
sudo make install
```
## Problems with the code

## TIPS :)
