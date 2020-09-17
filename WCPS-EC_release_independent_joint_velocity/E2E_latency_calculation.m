clear;
clc;

ft_size = 20;
bar_width = 0.6;


load('t_l_read_u.mat');
load('t_l_write_x.mat');
t_l_read_u_PID_l = t_l_read_u;
t_l_write_x_PID_l = t_l_write_x;

[seq_unique_o,idx_o] = unique(t_l_read_u_PID_l(:,1),'rows'); % OPT Linux
t_l_read_u_unique_o = t_l_read_u_PID_l(idx_o,:);
e2e_latency_pid = [];

i = 1;
while(i<=size(t_l_read_u_unique_o,1))
    if t_l_write_x_PID_l(i,1)~= t_l_read_u_unique_o(i)
        t_l_write_x_PID_l(i,:) = [];
        i = i;
    else
        i = i+1;
    end
end


for i = 2: size(t_l_read_u_unique_o,1)    
    e2e_latency_pid =[e2e_latency_pid; t_l_read_u_unique_o(i,6)*60+t_l_read_u_unique_o(i,7)-t_l_write_x_PID_l(i,6)*60-t_l_write_x_PID_l(i,7)];    
end


hFig=figure;
set(hFig, 'Units','normalized', 'Position', [0.1 0.1 0.65 0.5])
bh=boxplot([e2e_latency_pid]*1000,'width',bar_width);
%handle(bh)
for i=1:size(bh,1)
    set(bh(i,:),'linewidth',1.5);
end
set(gca,'XTick',[1]);
set(gca,'XTickLabel',{'PID_R'});
ylabel('E2E Latency (ms)');
set(gca, 'FontSize', ft_size);