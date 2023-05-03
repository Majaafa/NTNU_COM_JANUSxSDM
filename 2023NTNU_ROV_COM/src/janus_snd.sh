#! /bin/bash

cd NTNU_ROV_COM/lib/janus-c-3.0.5; git remote -v | wc -l
./janus-tx --config-file txcfg.ini --packet-cargo "$1"
exit
exit

