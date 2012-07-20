#!/bin/bash
export OPENMODELICALIBRARY=$PWD:$OPENMODELICALIBRARY
tmpDir=/tmp/$(whoami)
mkdir -p $tmpDir
cd $tmpDir
kill -q $(pgrep omc) $(pgrep OMShell)
OMShell-terminal
