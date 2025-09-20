#!/bin/bash

MY_PATH=`pwd`

while [ ! -e "build/COLCON_IGNORE" ]; do
  cd ..
  if [[ `pwd` == "/" ]]; then
    # we reached the root and didn't find the build/COLCON_IGNORE file - that's a fail!
    echo "Cannot compile, probably not in a workspace (if you want to create a new workspace, call \"colcon init\" in its root first)".
    exit 1
  fi
done

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

colcon test-result --delete-yes

colcon test --paths $MY_PATH/.. -p 1 --event-handlers console_direct+

colcon test-result --all --verbose
