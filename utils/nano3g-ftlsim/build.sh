#!/bin/bash
# Host harness for the iPod nano 3G FTL: builds firmware/target/arm/s5l8702/
# ipodnano3g/ftl-nano3g.c against a simulated NAND and runs write/sync/
# restore scenarios with a reference model. Usage: ./build.sh [-v]
set -e
cd "$(dirname "$0")"
RB=../../firmware/target/arm/s5l8702/ipodnano3g
gcc -std=gnu11 -O1 -g -Wall -Wno-unused-function -Wno-format -Wno-address-of-packed-member \
    -Iinc -I$RB -I. -DSIM_HARNESS harness.c sim_nand.c -o ftlsim
./ftlsim "$@"
