#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /app/install/local_setup.bash

exec "$@"
