# Wireless Cyber-Physical Simulator - Edge-Computing (WCPS-EC)
# Yehan Ma yehan.ma@wustl.edu 

(1) Install Robotics Toolbox version 10.3 
http://petercorke.com/wordpress/toolboxes/robotics-toolbox

(2) Brief note of the files:
A. Main file of running MATLAB/Simulink: p560_simulation_main_position.m
B. Simulink (SDTRT) model: ploop_test_modify2017.slx
C. Python client running on Windows (the server runs MATLAB): file_access_client.py
D. Recursive Discrete PID Controller running on remote server (Python): controller_py_MPC.py controller_py_PID.py 
E. State measurements file: sensor_data.txt
F. Current control command file: control_command.txt

(3) Instructions of operating WCPS-EC:
A. On remote server terminal: 
>> python controller_py_PID.py
or
>> python controller_py_MPC.py

B. On Windows terminal: change the IP address of Line25 to the IP address of remote server
>> python file_access_client.py

C. On Windows MATLAB command window: p560_simulation_main_position

Note: this is a initial version. We will keep updating it.

Please refer to and cite the following article -

Y. Ma, C. Lu, B. Sinopoli and S. Zeng, Exploring Edge Computing for Multi-Tier Industrial Control, IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems (TCAD), Special Issue on ESWEEK 2020 - Proceedings of ACM International Conference on Embedded Software (EMSOFT), September 2020.