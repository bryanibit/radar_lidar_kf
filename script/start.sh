#!/usr/bin/env sh
cd ../build
cmake ..
make -j;
./main
cd ../script;
sleep 3
./draw_output.py
