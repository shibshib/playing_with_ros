#!/bin/bash

python unit_tests.py
./run_bag_file.sh &
python node.py 30 x 100
