function write_x (x,yh,t)
global t_l_write_x delta_tp delta_r socket_counter sensor_seq
% t_write_x = tic; 
if rem(socket_counter,delta_tp/delta_r)==0
    t_l_write_x= [t_l_write_x;sensor_seq clock];
    fileID = fopen('sensor_data.txt','a');
    fprintf(fileID,'%011.6f, %011.6f, %011.6f, %011.6f, %011.6f, %011.6f\n',x(1),x(2),x(3),yh,sensor_seq,t);
    fclose(fileID);
    sensor_seq = sensor_seq + 1;
    
end
socket_counter = socket_counter+1;
end