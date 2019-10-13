#!/usr/bin/bash
# In Sphinx v1.4 --datalog is enabled by default

fdir="$(pwd)/$(dirname $0)"
cd $fdir

    # $SPHINX_ROOT/drones/local_bebop2.drone \
sphinx --datalog tmpBXsMv8.world \
    $SPHINX_ROOT/drones/local_anafi4k.drone \
    $SPHINX_ROOT/actors/pedestrian.actor::name=subject::path=tmpvGsKxl.path \
    $SPHINX_ROOT/actors/pedestrian.actor::name=pedestrian0::path=tmp12QhIR.path
    # Subject path always has a delayed start
