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

TEST_FILES=$(find . -name "*.test" -type f -printf "%f\n")

for TEST_FILE in `echo $TEST_FILES`; do

  echo "$0: running test '$TEST_FILE'"

  # folder for test results
  TEST_RESULT_PATH=$(realpath /tmp/$RANDOM)
  mkdir -p $TEST_RESULT_PATH

  # run the test
  rostest $PACKAGE $TEST_FILE $TEXT_OUTPUT --results-filename=$PACKAGE.test --results-base-dir="$TEST_RESULT_PATH"

  # evaluate the test results
  echo test result path is $TEST_RESULT_PATH
  catkin_test_results "$TEST_RESULT_PATH"

done
