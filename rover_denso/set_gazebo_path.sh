#!/bin/bash

MODEL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"/gazebo_models

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$MODEL_DIR

echo
echo GAZEBO_MODEL_PATH
echo $GAZEBO_MODEL_PATH

