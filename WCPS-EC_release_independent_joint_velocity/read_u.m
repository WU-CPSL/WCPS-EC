function u = read_u
global t_l_read_u
fileID = fopen('control_command.txt','r');
result = fscanf(fileID,'%f, %f');
if size(result,1)~= 2
    u = 0;
    seq = 0;
else
    u = result(1);
    seq = result(2);
    t_l_read_u =[t_l_read_u;seq clock];
end

fclose(fileID);

end