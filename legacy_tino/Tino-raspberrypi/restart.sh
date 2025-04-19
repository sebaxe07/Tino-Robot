#!/bin/bash

# kill current MAIN.PY
kill -9 main.py

# start new MAIN.PY
cd "$(dirname "$(readlink -f "$0")")"

# Activate the virtual environment
source  ~/etereum/bin/activate

# Execute the Python script
python3 main.py
