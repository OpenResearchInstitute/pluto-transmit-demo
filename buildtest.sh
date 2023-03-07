#!/bin/sh
gcc -L/usr/lib/arm-linux-gnueabihf/libiio.so -o iio-tx iio-tx.c -liio

