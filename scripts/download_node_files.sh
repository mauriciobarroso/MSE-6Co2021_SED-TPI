#!/bin/bash

URL="http://192.168.4.1"
NODE_FILES_PATH="../node_files"
LOG_FILE="log.txt"

# Get ESP32-S2 log file
echo "Running wget..."
wget -N "${URL}/${LOG_FILE}" -P "${NODE_FILES_PATH}"

# Get ESP32-S2 IMU data files
while IFS= read -r line
do
  if [[ ${line::1} == "A" ]]
  then
    echo 1;
    wget -N "${URL}/$line" -P "${NODE_FILES_PATH}"; fi
done < "${NODE_FILES_PATH}/${LOG_FILE}"