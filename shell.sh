#!/bin/bash
export OPENMODELICALIBRARY=$PWD:$OPENMODELICALIBRARY
tmpDir=/tmp/$(whoami)
mkdir -p $tmpDir
cd $tmpDir
OMShell-terminal
