%% Read Data
clear;clc;%clf;

%%%%%%%%%%%%%%%%% Choose file here, MATLAB autofills
A = readmatrix('/home/noel/Downloads/data_hardware.csv');
%%%%%%%%%%%%%%%%

% "t,contact,x,y,z,q_w,q_x,q_y,q_z,l,wheel_pos1,wheel_pos2,wheel_pos3,x_dot,y_dot,z_dot,w_1,w_2,w_3,l_dot,wheel_vel1,wheel_vel2,wheel_vel3,u_spring,tau_1,tau_2,tau_3,command_1,command_2,command_3"

ind = 1;
t = (A(:,ind)-A(1,ind));        ind=ind+1; % Sample time
contact = A(:,ind);             ind=ind+1;
pos = A(:,ind:ind+2);           ind=ind+3;
quat = A(:,ind:ind+3);          ind=ind+4;
tmp = quat;
quat(:,1) = tmp(:,4);
quat(:,2) = tmp(:,1);
quat(:,3) = tmp(:,2);
quat(:,4) = tmp(:,3);
l = A(:,ind);                   ind=ind+1;
wheel_pos = A(:,ind:ind+2);     ind=ind+3;
vel = A(:,ind:ind+2);           ind=ind+3;
omega = A(:,ind:ind+2);         ind=ind+3;
l_dot = A(:,ind);               ind=ind+1;
wheel_vel = A(:,ind:ind+2);     ind=ind+3;

torque = A(:,ind:ind+3);        ind=ind+4;

command = A(:,ind:ind+2);        ind=ind+3;
quat_des = A(:,ind:ind+3);        ind=ind+4;
tmp = quat_des;
quat_des(:,1) = tmp(:,4);
quat_des(:,2) = tmp(:,1);
quat_des(:,3) = tmp(:,2);
quat_des(:,4) = tmp(:,3);

%%
% clf

hfig1 = figure(1);
set(gcf,'renderer','painters')
hfig1.Position = [0 0 1700 1000];
set(hfig1,'WindowStyle','normal');
tg = uitabgroup(hfig1);

c3 = lines(3);
c4 = lines(4);


t_start = 0;
tabPlotSingle(tg,t-t_start,pos,'Pos')

% plot(t,contact)
tabPlotSingle(tg,t,quat,'Quat')
plot(t,quat_des,'--')
tabPlotSingle(tg,t-t_start,vel,'Vel')
tabPlotSingle(tg,t,quat*0,'d_bar')
tabPlotSingle(tg,t,omega,'Omega')
tabPlotSingle(tg,t,l,'Length')
tabPlotSingle(tg,t,torque,'Torque')
tabPlotSingle(tg,t,wheel_vel,'Wheel Vel')


%% 
fig_pos = [0 0 1000 1000];

function tabPlot(tg,t,dt,ph,x, x_pred,title)
h = uitab(tg, 'Title', title);
ax = axes('Parent', h);
hold on
c = lines(size(x,2));
for i = 1:size(x,2)
    plot(t,x(:,i),'color',c(i,:));
    plot(t+dt*ph,x_pred(:,i),'--','color','k');
end
grid on;
% for tau = 0:dt:t(end)
%     line([tau tau],ax.YLim,'color',[0 0 0 .1])
% %     xline(tau,'alpha',.1)
% end
end

function tabPlotSingle(tg,t,x,title)
h = uitab(tg, 'Title', title);
ax = axes('Parent', h);
hold on
c = lines(size(x,2));
for i = 1:size(x,2)
    plot(t,x(:,i),'color',c(i,:),'linewidth',3);
end
grid on;
% for tau = 0:dt:t(end)
%     line([tau tau],ax.YLim,'color',[0 0 0 .1])
% %     xline(tau,'alpha',.1)
% end
end
