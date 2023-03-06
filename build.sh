#!/usr/bin/bash

ODIN=odin

DEBUG="-o:minimal -use-separate-modules"
FAST="-o:speed"

set -e
$ODIN run . $FAST -show-timings -define:DEFAULT_TEMP_ALLOCATOR_BACKING_SIZE=20_000_000 
# ./plot.py 
