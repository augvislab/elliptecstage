#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 13 20:25:12 2021

@author: ejetzer
"""
import serial
from ellipticstage import ElloStage


with serial.Serial(port='COM3', baudrate=9600, timeout=0.2) as com:
    stage = ElloStage(com, 0)
    
    # One need to change motor parameters
    stage.initialize_motor()  # Default values are set
    
    # initialize the position
    stage.move_home()
    command, data, address = stage.read_message_blocking_position_response()
    print(f'{command}\t{data}\t{address}')
    
    # Move stage
    pos = 2.0  # [mm]
    stage.move_absolute(pos)
    command, z, address = stage.read_message_blocking_position_response()
    print(f'{command}\t{z}\t{address}')