#!/bin/bash

JANUSPATH=../lib/janus-c-3.0.5/bin/

echo $1
echo $2

cd $1

./janus-tx --config-file ../etc/txcfg_rawfile.ini --packet-cargo "$2"
