clear;
clc;
close all;
warning off;
figure;
iteration = 1;
ft_size = 20;
line_width = 2;
IAE_avg = [];
time_delay = 0;
global delta_t delta_r
 
fileID = fopen('sensor_data.txt','w');
fprintf(fileID,'%011.6f, %011.6f, %011.6f, %011.6f, %011.6f\n',0,0,0,0,1);
fclose(fileID)

fileID = fopen('control_command.txt','w');
fprintf(fileID,'%3.5f, %3.5f\n',0,1)
fclose(fileID)
%% Experiment parameters
round_count=1;      %round number of simulations
Tf =0.5; 
delta_t=1/200;      %control period
delta_r =1/1000;    %discrete period
Cost=[];

%% PID controller, Kp and Ki are defined in Simulink Block
Kd = 0.0006;

%% Policy iteration parameters
global t_l_read_u t_l_write_x sensor_seq socket_counter
sensor_seq = 1;
t_l_read_u = [];
t_l_write_x = [];

%% RUN Wireless  Process Control Simulation
colormap(parula)
for yh=1:round_count
socket_counter = 1;

fileID = fopen('sensor_data.txt','w');
fprintf(fileID,'%011.6f, %011.6f, %011.6f, %011.6f, %011.6f\n',0,0,0,0,1);
fclose(fileID);

fileID = fopen('control_command.txt','w');
fprintf(fileID,'%3.5f, %3.5f\n',0,1);
fclose(fileID);

tstart = tic;       
option = simset('solver','ode4','FixedStep',delta_r);
simulation_results = sim('vloop_test_modify2017.slx');
toc(tstart)

%% Plot results
TIME = simulation_results.TIME.Data;
w = simulation_results.w.Data;
w_ref = simulation_results.w_ref.Data;
uu = simulation_results.u.Data;
uu1 = simulation_results.u1.Data;

%% velocity
plot(TIME,w,'LineWidth',line_width);
hold on;
plot(TIME, w_ref, 'g--','LineWidth',line_width);
werror=sum(abs(w-w_ref))/size(w_ref,1);
Cost_round=werror
Cost=[Cost;Cost_round];

end

legend('w','w*');
set(gca, 'FontSize', ft_size);
xlabel('t (s)', 'FontSize',ft_size);
ylabel('Omega (rad/s)', 'FontSize',ft_size);
IAE = sum(Cost)/round_count
IAE_avg = [IAE_avg;time_delay IAE];

figure;
plot(TIME,uu,'LineWidth',line_width);
hold on;
plot(TIME, uu1,'LineWidth',line_width);
legend('u\_matlab','u\_remote');
set(gca, 'FontSize', ft_size);
xlabel('t (s)', 'FontSize',ft_size);
ylabel('Voltage (V)', 'FontSize',ft_size);
