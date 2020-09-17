#!/usr/bin/env 
# coding: utf-8
import sys
import socket
import os
import time
import math
import numpy
from datetime import datetime
last_num_lines = 0
num_lines = 0
while True:
  last_num_lines = num_lines
  num_lines = 0
  with open('sensor_data.txt', 'r') as sensor_file:
    for line in sensor_file:
          num_lines += 1
  
  if last_num_lines!=num_lines and num_lines>0:
    with open('sensor_data.txt', 'r') as sensor_measurement:
      sensor_data = (list(sensor_measurement)[-1])
    # print sensor_data
    sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server_address = ('172.16.11.89',8000) #IP address of remote server
    sock.connect(server_address)
    sock.sendall(sensor_data)
    control_command = sock.recv(30)
    if control_command:
      # print control_command
      command_file = open('control_command.txt', 'w')              
      command_file.write(control_command)
      command_file.close()
    sock.close()
        