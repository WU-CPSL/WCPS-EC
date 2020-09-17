# Wireless Cyber-Physical Simulator - Edge-Computing (WCPS-EC)
# Yehan Ma yehan.ma@wustl.edu 

(1) Install Robotics Toolbox version 10.3 
http://petercorke.com/wordpress/toolboxes/robotics-toolbox

(2) Brief note of the files:
A. Main file of running MATLAB/Simulink: p560_simulation_main_velocity2017.m
B. Simulink (SDTRT) model: vloop_test_modify2017.slx, and the corresponding S-functions in Simulink:
   (a) Plant model: JointSfun.m
   (b) write state measurements to file: write_x.m
   (c) read control commands from file: read_u.m
   (d) controller code in MATLAB (redundant): Constrollersfun.m
C. Python client running on Windows (the server runs MATLAB): file_access_client.py
D. Recursive Discrete PID Controller running on remote server (Python): controller_py_PID.py 
E. State measurements file: sensor_data.txt
F. Current control command file: control_command.txt
G. Example code of E2E latency measurements: E2E_latency_calculation.m

(3) Instructions of operating WCPS-EC:
A. On remote server terminal: python controller_py_PID.py
B. On Windows terminal: change the IP address of Line24 to the IP address of remote server
python file_access_client.py
C. On Windows MATLAB command window: p560_simulation_main_velocity2017

Note: this is a initial version. We will keep updating it.

Please refer to and cite the following article -

Y. Ma, C. Lu, B. Sinopoli and S. Zeng, Exploring Edge Computing for Multi-Tier Industrial Control, IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems (TCAD), Special Issue on ESWEEK 2020 - Proceedings of ACM International Conference on Embedded Software (EMSOFT), September 2020.