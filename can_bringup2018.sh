#!/bin/bash

sudo ip link set can0 type can bitrate 1000000 triple-sampling on
sudo ip link set can0 up

