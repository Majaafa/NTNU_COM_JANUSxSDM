#!/bin/bash

SDMSHPATH=../sdmsh/
JANUSPATH=../janus-c-3.0.5/bin/

$SDMSHPATH./sdmsh 189 -e "stop;config 30 0 3 0"
$SDMSHPATH./sdmsh 198 -e "stop;config 30 0 3 0"