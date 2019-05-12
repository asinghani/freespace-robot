#!/bin/bash
arduino-cli compile --fqbn arduino:avr:uno .
arduino-cli upload -p $1 --fqbn arduino:avr:uno .
