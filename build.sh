#!/usr/bin/bash

ODIN=odin

set -e
$ODIN run . -o:speed  -show-timings -no-bounds-check -define:DEFAULT_TEMP_ALLOCATOR_BACKING_SIZE=20_000_000 
# ./plot.py 
