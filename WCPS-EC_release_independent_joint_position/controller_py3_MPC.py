#!/usr/bin/env python
import sys
import socket
import time
import numpy
from datetime import datetime
from cvxopt import matrix, solvers

round_counter = 1
round_count = 0
counter = 1
empty_flag = 0
theta_ref = 0
thetaini = 0
uini = 0
delta_t = 0.05
MR = 0.000002
MQ = numpy.array([[30]])
MS = numpy.array([[60]])
Nstates = 1
Ninputs = 1
Horizon = 30

Oidxs = 1
Nopt = Nstates * Horizon + Ninputs * (Horizon - 1)

yh_last = 0
yh = 1
compu_latency = []

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('0.0.0.0', 8000)
print('starting up on %s port %s' % server_address, file=sys.stderr)
sock.bind(server_address)
sock.listen(1)
while True:
    connection, client_address = sock.accept()
    print('Connection from:', client_address, file=sys.stderr)
    try:
        while True:
            data = connection.recv(80)
            if data:
                data = data.decode("utf-8")
                t = time.time()
                print(data)
                print(round_counter)
                timeT = datetime.now().strftime('%Y,%m,%d,%H,%M,%S.%f')
                print("data=", data)
                data_split = [float(x.strip()) for x in data.split(',')]

                # print data
                if data_split[0] != 10000:
                    theta_ref = data_split[0]
                    theta = data_split[1]
                    theta_error = data_split[2]
                    state = numpy.array([[theta]])
                    SSMA = numpy.array([[1]], dtype=numpy.float64)
                    SSMB = numpy.array([[delta_t * 1 / 107.815]], dtype=numpy.float64)
                    SSMC = numpy.array([1])
                    H = numpy.zeros((Nopt, Nopt))
                    f = numpy.zeros((1, Nopt))
                    Aeq = numpy.zeros((Nstates * Horizon, Nopt))
                    beq = numpy.zeros((Nstates * Horizon, 1))
                    lb = numpy.zeros((Nopt, 1))
                    ub = numpy.zeros((Nopt, 1))
                    z = numpy.zeros((Nopt, 1))
                    o1 = numpy.dot(SSMC, MQ)
                    o2 = numpy.dot(o1, numpy.transpose(SSMC))
                    H_diag = 2 * o2
                    o3 = numpy.dot(theta_ref, SSMC)
                    o4 = numpy.dot(o3, MQ)
                    f_assign = numpy.dot(-2, o4)
                    o5 = numpy.dot(o3, MS)
                    f_assignt = numpy.dot(-2, o5)
                    o6 = numpy.dot(SSMC, MS)
                    o7 = numpy.dot(o6, numpy.transpose(SSMC))
                    H_diagt = 2 * o7

                    for k in range(1, Horizon):
                        H[(k - 1) * Nstates + Oidxs - 1, (k - 1) * Nstates + Oidxs - 1] = H_diag
                        H[Nstates * Horizon + (k - 1) * Ninputs + 1 - 1, Nstates * Horizon + (
                                    k - 1) * Ninputs + 1 - 1] = 2 * MR
                        f[0, (k - 1) * Nstates:(k - 1) * Nstates + Nstates] = f_assign

                    H[(Horizon - 1) * Nstates + Oidxs - 1, (Horizon - 1) * Nstates + Oidxs - 1] = H_diagt
                    f[0, (Horizon - 1) * Nstates:(Horizon - 1) * Nstates + Nstates] = f_assignt

                    for k in range(1, Horizon):
                        Aeq[(k - 1) * Nstates:(k - 1) * Nstates + Nstates,
                        (k - 1) * Nstates:(k - 1) * Nstates + Nstates] = -SSMA
                        Aeq[(k - 1) * Nstates:(k - 1) * Nstates + Nstates,
                        k * Nstates:k * Nstates + Nstates] = numpy.eye(Nstates)
                        Aeq[(k - 1) * Nstates:(k - 1) * Nstates + Nstates,
                        Nstates * Horizon + (k - 1) * Ninputs + 1 - 1] = numpy.transpose(-SSMB)

                    Aeq[(Horizon - 1) * Nstates:(Horizon - 1) * Nstates + Nstates, 0:Nstates] = numpy.eye(Nstates)
                    beq[(Horizon - 1) * Nstates:(Horizon - 1) * Nstates + Nstates] = state

                    for k in range(1, Horizon + 1):
                        lb[(k - 1) * Nstates:(k - 1) * Nstates + Nstates] = -100000
                        ub[(k - 1) * Nstates:(k - 1) * Nstates + Nstates] = 100000

                    for k in range(1, Horizon):
                        ub[Nstates * Horizon + (k - 1) * Ninputs + 1 - 1] = 100000
                        lb[Nstates * Horizon + (k - 1) * Ninputs + 1 - 1] = -100000

                    yh_last = yh
                    yh = data_split[3]
                    sensor_seq = data_split[4]
                    if yh_last != yh:
                        round_counter = 1
                        yh_last = yh

                    P = matrix(H, tc='d')
                    q = matrix(numpy.transpose(f), tc='d')
                    eye_matrix = numpy.eye(Nopt)
                    G = matrix(numpy.concatenate((eye_matrix, -eye_matrix), axis=0), tc='d')
                    h = matrix(numpy.concatenate((ub, -lb), axis=0), tc='d')
                    Aeqq = matrix(Aeq, tc='d')
                    beqq = matrix(beq, tc='d')
                    sol = solvers.qp(P, q, G, h, Aeqq, beqq)
                    z = sol['x']

                    if round_counter == 2 or round_counter == 1:
                        uout = uini
                    else:
                        uout = z[Nstates * Horizon + 1 - 1]

                    print("Uout generated by python code on server:", uout)
                    counter = counter + 1
                    connection.send(('% 3.4f,% 3.4f' % (uout, sensor_seq)).encode("utf-8"))
                    round_counter = round_counter + 1
                    elapsed = time.time() - t
                    print("Computational latency:", elapsed)
                    compu_latency.extend([elapsed])
                    if sensor_seq > 1198:
                        print("Computational latency list", compu_latency)

            else:
                break

    finally:
        connection.close()
