#!/bin/bash

ip route add 10.0.0.7 via 10.0.0.6
cd build && ./hopper_ctrl_hardware
