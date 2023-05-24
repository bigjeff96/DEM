#!/usr/bin/bash

ODIN=odin

DEBUG="-o:none -debug -use-separate-modules"
FAST="-o:speed -no-bounds-check"

set -ex
echo BUILD:
time $ODIN build . $DEBUG -define:DEFAULT_TEMP_ALLOCATOR_BACKING_SIZE=20_000_000                                        
echo OUTPUT:
./master_thesis.bin
