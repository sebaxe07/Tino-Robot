#!/bin/bash
echo "Compiling and uploading new_base_tino sketch..."
arduino-cli compile -b arduino:avr:mega new_base_tino && arduino-cli upload -p /dev/ttyBASE -b arduino:avr:mega new_base_tino
echo "Done!"
