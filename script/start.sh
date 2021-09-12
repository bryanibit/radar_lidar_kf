#!/usr/bin/env sh
cd ../build
make -j;
./main
cd ../script;
./draw_output.py
