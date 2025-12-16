#!/bin/bash

# Check if directory with launch files is provided
if [ -z "$1" ]; then
    exit 0
fi

# Check if provided directory exists
if [ ! -d "$1" ]; then
    echo "The provided directory with launch files does not exist."
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
INPUT_DIR="${1%/}" # remove trailing slash if set
OUTPUT_DIR="$SCRIPT_DIR/../multisensor_calibration/launch"

# Copy files from input directory to output directory
if [ "$INPUT_DIR" != "$OUTPUT_DIR" ]; then
    cp -r "$INPUT_DIR/." "$OUTPUT_DIR" 2>/dev/null
fi

exit 0