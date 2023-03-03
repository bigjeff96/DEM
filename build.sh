#!/usr/bin/bash

ODIN=odin
# ODIN_ROOT="/home/joseph/Dropbox/Projects/odin/Odin-dev-2023-02/Odin-dev-2023-02/"

set -e
$ODIN run . -o:minimal -use-separate-modules  -show-timings -no-bounds-check -define:DEFAULT_TEMP_ALLOCATOR_BACKING_SIZE=20_000_000 
# $ODIN check .
# $ODIN build . -o:speed  -show-timings -define:DEFAULT_TEMP_ALLOCATOR_BACKING_SIZE=20_000_000 -no-bounds-check -out:delta_study.bin
# ./plot.py 

# using seperate modules causes the thing to not  for some reason

# to increase the temp alloc -define:DEFAULT_TEMP_ALLOCATOR_BACKING_SIZE=20_000_000
