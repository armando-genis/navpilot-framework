#!/bin/bash

# Check if directory with robot workspaces is provided
if [ -z "$1" ]; then
    exit 0
fi

# Check if provided directory exists
if [ ! -d "$1" ]; then
    echo "The provided directory with robot workspaces does not exist."
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
INPUT_DIR="${1%/}" # remove trailing slash if set
OUTPUT_DIR="$SCRIPT_DIR/../multisensor_calibration/resources/robot_workspaces"
OUTPUT_FILE="$SCRIPT_DIR/../multisensor_calibration/resources/robot_workspaces.qrc"

# Copy files from input directory to output directory
if [ "$INPUT_DIR" != "$OUTPUT_DIR" ]; then
    cp -r "$INPUT_DIR/." "$OUTPUT_DIR" 2>/dev/null
fi


# Write XML header
echo "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" > "$OUTPUT_FILE"
echo "<RCC>" >> "$OUTPUT_FILE"
echo "    <qresource prefix=\"/\">" >> "$OUTPUT_FILE"

# Find all ini files within directory and write into output
find "$OUTPUT_DIR" -type f -name "*.ini" | while read -r FILE; do
    REL_PATH="${FILE#$OUTPUT_DIR/}"
    echo "        <file>robot_workspaces/$REL_PATH</file>" >> "$OUTPUT_FILE"
done

# Write XML footer
echo "    </qresource>" >> "$OUTPUT_FILE"
echo "</RCC>" >> "$OUTPUT_FILE"

exit 0