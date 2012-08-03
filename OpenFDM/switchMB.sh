#!/bin/bash

usage="$0 [omc | std]"
files=`find . -regex ".*\.mos?"`

if [[ $# != 1 ]]
then
    echo $usage
elif [[ "$1" == "omc" ]]
then
    sed -i s/ModelicaOmc/MultiBodyOmc/g $files
    sed -i s/Modelica.Mechanics.MultiBody/MultiBodyOmc/g $files
elif [[ "$1" == "std" ]]
then
    sed -i s/MultiBodyOmc/Modelica.Mechanics.MultiBody/g $files
fi
