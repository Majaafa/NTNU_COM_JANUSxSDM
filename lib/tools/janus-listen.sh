#!/bin/bash

JANUSPATH=../janus-c-3.0.5/bin/

cd $1

exec ./janus-rx --config-file ../etc/rxcfg.ini --verbose 1 --stream-driver raw --stream-driver-args ../data/janusMessage.raw 2> ../data/decodedJanus.txt

