clear;
clc;
warning off;
run('/Users/yehanspc/Codes/rvctools/startup_rvc.m');
% figure;
iteration = 1;
ft_size = 20;
line_width = 2;
load('q0_m1.mat');
time_delay = 0;
global delta_t delta_r delta_tp u_last
u_last = 0;

fileID = fopen('sensor_data.txt','w');
fprintf(fileID,'%011.6f, %011.6f, %011.6f, %011.6f, %011.6f, %011.6f\n',6,6,0,0,1,0);
fclose(fileID)

fileID = fopen('control_command.txt','w');
fprintf(fileID,'%3.5f, %3.5f\n',0,1)
fclose(fileID)

%% Experiment parameters
Tf =8;
delta_t=1/500; % velocity control period  
delta_tp = 1/20; % position control period 
delta_r =1/500; % discrete period
Kg  = 1/107.815;
%% MPC Controller parameters
MPCHorizon=10;
Q=30;
R=3;
S=600;

Qp=30;
Rp=0.000002;
Sp=60;

%% PI position controller
Kd = 0.0006;

%% PID position controller

Kp = 800;
Kii = 0;
Kdd = 0;

%% Policy iteration parameters
global uuu t_l_read_u t_l_write_x sensor_seq socket_counter socket_counter2
sensor_seq = 1;
socket_counter = 1;
socket_counter2 = 1;
t_l_read_u = [];
t_l_write_x = [];
uuu = [];

%% RUN Wireless  Process Control Simulation
q0 = q0_m(yh);
qf = 8;

tstart = tic;       
option = simset('solver','ode4','FixedStep',delta_r);
simulation_results = sim('ploop_test_modify2017.slx');
toc(tstart)

%% Plot results
TIME = simulation_results.TIME.Data;
w = simulation_results.w.Data;
w_ref = simulation_results.w_ref.Data;


%% position
theta = simulation_results.theta.Data;
theta_ref = simulation_results.theta_ref.Data;
plot(TIME,theta,'LineWidth',line_width);
hold on;
plot(TIME, theta_ref, 'g--','LineWidth',line_width);
thetaerror=sum(abs(theta-theta_ref))/size(theta_ref,1);
thetaerror_1=sum(abs(theta(1:500)-theta_ref(1:500)))/500;
thetaerror_2=sum(abs(theta(501:2500)-theta_ref(501:2500)))/2000;
thetaerror_3=sum(abs(theta(2501:4001)-theta_ref(2501:4001)))/1501;
Cost_round=thetaerror;

legend('theta','theta*');
set(gca, 'FontSize', ft_size);
xlabel('t (s)', 'FontSize',ft_size);
ylabel('Theta (rad)', 'FontSize',ft_size);
