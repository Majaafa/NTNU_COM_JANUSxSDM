#!/bin/bash

SDMSHPATH=../sdmsh/
JANUSPATH=../janus-c-3.0.5/bin/

cd $SDMSHPATH

exec ./sdmsh 198 -e "stop;config 30 0 3 0"

exec ./sdmsh 198 -e "rx 0 connect:127.0.0.1:9994"
