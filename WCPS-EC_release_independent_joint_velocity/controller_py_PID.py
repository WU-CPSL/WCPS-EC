#!/usr/bin/env python
import sys
import socket
import os
import time
import serial
import math
import numpy
from datetime import datetime
from threading import Thread
from SocketServer import ThreadingMixIn

round_counter = 1  # A counter for running multiple rounds of simulations
w_ref = 50         # reference value of joint velocity
wini = 0           # initial value of joint velocity
uini = 0           # initial control command
delta_t = 0.005    # discrete period 
yh_last = 0        # last round number
yh = 1             # current round number
kp = 0.3           # proportion coefficient of PID
ki = 0.2           # integral coefficient of PID
kd = 0.0006        # differential coefficient of PID
w_last = wini      
w_last_last = wini
uout_last = uini

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('0.0.0.0', 8000) #IP address of remote server
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)
sock.listen(1)
while True:
    connection, client_address = sock.accept()
    print >>sys.stderr,'Connection from:', client_address
    try:
      while True:
            data=connection.recv(64)     
            if data:
                  timeT=datetime.now().strftime('%Y,%m,%d,%H,%M,%S.%f')
                  print "data=",data
                  data_split = [float(x) for x in data.split(',')]

                  if data_split[0] !=10000:
                    w_ref = data_split[0]
                    w = data_split[1]
                    w_error = data_split[2]


                    # if round_counter == 2 or round_counter == 1:
                    #   w_last_last = w
                    #   w_last = w
                    #   uout_last = uini
                      

                    yh_last = yh
                    yh = data_split[3]
                    sensor_seq = data_split[4]
                    if yh_last!=yh:
                      round_counter = 1
                      yh_last = yh

  
                    if round_counter == 1:
                      w_last_last = wini
                      w_last = wini
                      uout_last = uini
                      #uout = uini
                      uout = uout_last + kp*(w_error - w_last) + ki*delta_t*w_error + kd*(w_error-2 * w_last + w_last_last)/delta_t

                    else:
                      uout = uout_last + kp*(w_error - w_last) + ki*delta_t*w_error + kd*(w_error-2 * w_last + w_last_last)/delta_t

                    # print "round_counter",round_counter
                    # print "uout_last", uout_last
                    # print "deltaP", kp*(w_error - w_last)
                    # print "deltaI", ki*delta_t*w_error
                    # print "deltaD", kd*(w_error-2 * w_last + w_last_last)/delta_t
                    # print "w_error",w_error
                    # print "w_last",w_last
                    # print "w_last_last",w_last_last


                    uout_last = uout
                    w_last_last = w_last
                    w_last = w_error
                    

                    print "Uout generated by python code on server:", uout   
                    connection.send('% 3.4f,% 3.4f' % (uout, sensor_seq))
                    round_counter = round_counter + 1

            else:
                  break

    finally:
        connection.close()
        
