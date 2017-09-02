#!/bin/bash
cd /project
mkdir build
cd build
cmake .. && make && ./pid
