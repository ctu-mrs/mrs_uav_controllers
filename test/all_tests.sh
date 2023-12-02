#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

PACKAGE="mrs_uav_controllers"
VERBOSE=1

[ "$VERBOSE" = "0" ] && TEXT_OUTPUT=""
[ "$VERBOSE" = "1" ] && TEXT_OUTPUT="-t"

# build the package
catkin build $PACKAGE # it has to be fully built normally before building with --catkin-make-args tests
catkin build $PACKAGE --no-deps --catkin-make-args tests

catkin test --this -i
