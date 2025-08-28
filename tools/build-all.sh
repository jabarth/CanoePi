#!/bin/bash
#
# This script builds all the container images defined in the root compose.yaml file.
#

set -e

echo "Building all CanoeDash container images..."
podman-compose --in-project-name canoedash build

echo "Build complete."
