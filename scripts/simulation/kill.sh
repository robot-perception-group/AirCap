#!/bin/bash

sleep 120
rosnode kill /my_bag
rosnode kill /my_image_bag
./cleanup.sh
