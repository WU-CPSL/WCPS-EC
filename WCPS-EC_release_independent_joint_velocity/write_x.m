function write_x (x,yh)
global t_l_write_x delta_t delta_r socket_counter sensor_seq 
if rem(socket_counter,delta_t/delta_r)==0
    t_l_write_x= [t_l_write_x;sensor_seq clock];
    fileID = fopen('sensor_data.txt','a');
    fprintf(fileID,'%011.6f, %011.6f, %011.6f, %011.6f, %011.6f\n',x(1),x(2),x(3),yh,sensor_seq);
    fclose(fileID);
    sensor_seq = sensor_seq + 1;
    
end

socket_counter = socket_counter+1;

end