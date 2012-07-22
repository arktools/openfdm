#!/bin/bash
export OPENMODELICALIBRARY=$PWD:$OPENMODELICALIBRARY
tmpDir=/tmp/$(whoami)
mkdir -p $tmpDir
cd $tmpDir
pids=`pgrep "omc|OMShell|OMPlot"`
if [[ "$pids" != "" ]]
then
    echo killing: $pids
    kill $pids
else
    echo no omc processes to kill
fi
OMShell-terminal
