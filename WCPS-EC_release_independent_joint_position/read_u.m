function u = read_u
global t_l_read_u u_last
fileID = fopen('control_command.txt','r');
result = fscanf(fileID,'%f, %f');
if size(result,1)~= 2
    u = u_last;
    seq = 0;
else
    u = result(1);
    seq = result(2);
    u_last = u;
    t_l_read_u =[t_l_read_u;seq clock];
end

fclose(fileID);

end