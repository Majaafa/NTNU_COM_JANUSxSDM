#! /bin/bash

cd NTNU_ROV_COM/lib/sdmsh
./sdmsh "$1" -e "tx 102400 tcp:connect:127.0.0.1:9999"
exit
exit
