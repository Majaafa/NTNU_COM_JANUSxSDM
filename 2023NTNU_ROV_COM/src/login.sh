#! /bin/bash

gnome-terminal -- bash -c "sshpass -p passord1 ssh -t janus@192.168.0.18 'bash janus_snd.sh \"$1\"; bash -l'; exec bash"
gnome-terminal -- bash -c "sshpass -p passord1 ssh -t janus@192.168.0.18 'bash sdmsh_snd.sh $2; bash -l'; exec bash"
