#!/usr/bin/bash

ODIN=odin

DEBUG="-o:minimal -debug -use-separate-modules -thread-count:8"
FAST="-o:speed -no-bounds-check"

set -e
echo BUILD:
time $ODIN build . $DEBUG -define:DEFAULT_TEMP_ALLOCATOR_BACKING_SIZE=20_000_000
echo OUTPUT:
./master_thesis.bin
