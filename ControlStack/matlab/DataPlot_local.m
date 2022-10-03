%% Read Data
clear;clc;clf;
A = readmatrix('../data/data.csv');
param = yaml.loadFile("../config/gains.yaml");

dt = param.Debug.dt;
N = param.MPC.N;
% MPC_dt = param.MPC.dt;
ph = param.Debug.predHorizon;

nx = 20;

ind = 1;
t = (A(:,ind)-A(1,ind));        ind=ind+1; % Sample time
contact = A(:,ind);             ind=ind+1;
pos = A(:,ind:ind+2);           ind=ind+3;
quat = A(:,ind:ind+2);          ind=ind+3;
l = A(:,ind);                   ind=ind+1;
wheel_pos = A(:,ind:ind+2);     ind=ind+3;
vel = A(:,ind:ind+2);           ind=ind+3;
omega = A(:,ind:ind+2);         ind=ind+3;
l_dot = A(:,ind);               ind=ind+1;
wheel_vel = A(:,ind:ind+2);     ind=ind+3;

torque = A(:,ind:ind+3);        ind=ind+4;
% state_pred = A(:,ind:ind+20);   ind=ind+20;
t_MPC = A(:,ind);               ind=ind+1;
sol = A(:,ind:ind+20*N+4*(N-1)-1); ind=ind+20*N+4*(N-1);
wasMPCupdated = A(:,ind);       ind=ind+1;

% pos_pred = state_pred(:,1:3);
% quat_pred = state_pred(:,4:7);
% l_pred = state_pred(:,8);
% wheel_pred = state_pred(:,9:11);
% vel_pred = state_pred(:,12:14);
% omega_pred = state_pred(:,15:17);
% l_vel_pred = state_pred(:,18);
% wheel_omega_pred = state_pred(:,19:20);

ind_pos = [];
ind_quat = [];
ind_l = [];
ind_w = [];
ind_vel = [];
ind_omega = [];
ind_l_vel = [];
ind_wheel_omega = [];
for i = 1:N
    ind_pos = [ind_pos (1:3)+(i-1)*20];
    ind_quat = [ind_quat (4:6)+(i-1)*20];
    ind_l = [ind_l (7)+(i-1)*20];
    ind_w = [ind_w (8:10)+(i-1)*20];
    ind_vel = [ind_vel (11:13)+(i-1)*20];
    ind_omega = [ind_omega (14:16)+(i-1)*20];
    ind_l_vel = [ind_l_vel (17)+(i-1)*20];
    ind_wheel_omega = [ind_wheel_omega (18:20)+(i-1)*20];
end

mpc_ind = find(wasMPCupdated(1:end-N)>0);

tau = [];
tau_u = [];
for i = 1:size(mpc_ind,1)-N
    tau = [tau t_MPC(mpc_ind(i:(i+N-1)))];
    tau_u = [tau_u t_MPC(mpc_ind(i:(i+N-2)))];
end

mpc_ind = mpc_ind(1:end-N);
% tau = tau-0.005;
% tau_u = tau_u-0.005;


% mpc_ind(1) = 0;
% mpc_ind = mpc_ind+1;

% tau = tau(:,mpc_ind)-2*dt;
% tau_u = tau_u(:,mpc_ind)-2*dt;

pos_MPC = sol(mpc_ind,ind_pos);
quat_MPC = sol(mpc_ind,ind_quat);
l_MPC = sol(mpc_ind,ind_l);
wheel_MPC = sol(mpc_ind,ind_w);
vel_MPC = sol(mpc_ind,ind_vel);
omega_MPC = sol(mpc_ind,ind_omega);
l_vel_MPC = sol(mpc_ind,ind_l_vel);
wheel_omega_MPC = sol(mpc_ind,ind_wheel_omega);
torque_MPC = sol(mpc_ind,20*N+1:end);



%%
clf

hfig1 = figure(1);
set(hfig1,'WindowStyle','normal');
tg = uitabgroup(hfig1);

c3 = lines(3);
c4 = lines(4);

tabPlotSingle(tg,t,dt,pos,'Pos')
% tabPlot(tg,t,dt,ph,pos,pos_pred,'Pos')
for i =1:3
    tmp_y = [pos_MPC(:,i:3:end)'; nan(1,size(tau,2))];
    tmp_x = [tau;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
end
a=gca;
a.XLim = [0,a.XLim(2)];
% plot(t,contact)
tabPlotSingle(tg,t,dt,quat,'Quat')
for i =1:3
    tmp_y = [quat_MPC(:,i:3:end)'; nan(1,size(tau,2))];
    tmp_x = [tau;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c4(i,:))
end
a=gca;
a.XLim = [0,a.XLim(2)];
tabPlotSingle(tg,t,dt,vel,'Vel')
for i =1:3
    tmp_y = [vel_MPC(:,i:3:end)'; nan(1,size(tau,2))];
    tmp_x = [tau;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
end
% for i =1:3
%     for j = 1:N
%         vel_error(j,:,i) = vel_MPC(:,i+3*(j-1))' - interp1(t,vel(:,i),tau(j,:));
%     end
% end
% tabPlotSingle(tg,t,dt,0*vel,'Vel_error')
% for i =1:3
%     tmp_y = [vel_error(:,:,i); nan(1,size(tau,2))];
%     tmp_x = [tau;  nan(1,size(tau,2))];
%     plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
% end


a=gca;
a.XLim = [0,a.XLim(2)];
tabPlotSingle(tg,t,dt,omega,'Omega')
for i =1:3
    tmp_y = [omega_MPC(:,i:3:end)'; nan(1,size(tau,2))];
    tmp_x = [tau;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
end
a=gca;
a.XLim = [0,a.XLim(2)];
tabPlotSingle(tg,t,dt,l,'Length')
tmp_y = [l_MPC(:,1:end)'; nan(1,size(tau,2))];
tmp_x = [tau;  nan(1,size(tau,2))];
plot(tmp_x(:), tmp_y(:),'--','color',c3(1,:))
a=gca;
a.XLim = [0,a.XLim(2)];
tabPlotSingle(tg,t,dt,torque,'Torque')
for i =1:3
    tmp_y = [torque_MPC(:,(i+1):4:end)'; nan(1,size(tau_u,2))];
    tmp_x = [tau_u;  nan(1,size(tau_u,2))];
    stairs(tmp_x(:), tmp_y(:),'--','color',c4(i+1,:))
end
a=gca;
a.XLim = [0,a.XLim(2)];
tabPlotSingle(tg,t,dt,wheel_vel,'Wheel Vel')
for i =1:3
    tmp_y = [wheel_omega_MPC(:,i:3:end)'; nan(1,size(tau,2))];
    tmp_x = [tau;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
end
a=gca;
a.XLim = [0,a.XLim(2)];



function tabPlot(tg,t,dt,ph,x, x_pred,title)
h = uitab(tg, 'Title', title);
ax = axes('Parent', h);
hold on
c = lines(size(x,2));
for i = 1:size(x,2)
    plot(t,x(:,i),'color',c(i,:));
    plot(t+dt*ph,x_pred(:,i),'--','color','k');
end
% for tau = 0:dt:t(end)
%     line([tau tau],ax.YLim,'color',[0 0 0 .1])
% %     xline(tau,'alpha',.1)
% end
end

function tabPlotSingle(tg,t,dt,x,title)
h = uitab(tg, 'Title', title);
ax = axes('Parent', h);
hold on
c = lines(size(x,2));
for i = 1:size(x,2)
    plot(t,x(:,i),'color',c(i,:));
end
% for tau = 0:dt:t(end)
%     line([tau tau],ax.YLim,'color',[0 0 0 .1])
% %     xline(tau,'alpha',.1)
% end
end
