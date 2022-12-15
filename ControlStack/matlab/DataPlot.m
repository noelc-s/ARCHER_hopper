%% Read Data
clear;clc;%clf;
A = readmatrix('../data/data.csv');
% A = readmatrix('../data/data_hardware.csv');
% A = readmatrix('../data/data_hardware_test39.csv');
sim = true;
if sim
    param = yaml.loadFile("../config/gains.yaml");
else
    param = yaml.loadFile("../config/gains_hardware.yaml");
end


% dt = param.Debug.dt;
dt = 0.01;
N = param.MPC.N;
% MPC_dt = param.MPC.dt;
% ph = param.Debug.predHorizon;

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
% state_pred = A(:,ind:ind+20);   ind=ind+21;
t_MPC = A(:,ind);               ind=ind+1;
sol = A(:,ind:ind+21*N+4*(N-1)-1); ind=ind+21*N+4*(N-1);
wasMPCupdated = A(:,ind);       ind=ind+1;
elapsed_time = A(:,ind:(ind+N-1));       ind=ind+N;
d_bar = A(:,ind:(ind+N-2));       ind=ind+N-1;

if sim
    command_d = A(:,ind:ind+1);    ind = ind+2;
else
    quat_d = A(:,ind:ind+3);         ind=ind+4;
    omega_d = A(:,ind:ind+2);         ind=ind+3;
end

elapsed_time = elapsed_time+t;

% pos_pred = state_pred(:,1:3);
% quat_pred = state_pred(:,4:7);make
% l_pred = state_pred(:,8);
% wheel_pred = state_pred(:,9:11);
% vel_pred = state_pred(:,12:14);
% omega_pred = state_pred(:,15:17);
% l_vel_pred = state_pred(:,18);
% wheel_omega_pred = state_pred(:,19:21);

ind_pos = [];
ind_quat = [];
ind_l = [];
ind_w = [];
ind_vel = [];
ind_omega = [];
ind_l_vel = [];
ind_wheel_omega = [];
for i = 1:N
    ind_pos = [ind_pos (1:3)+(i-1)*21];
    ind_quat = [ind_quat (4:7)+(i-1)*21];
    ind_l = [ind_l (8)+(i-1)*21];
    ind_w = [ind_w (9:11)+(i-1)*21];
    ind_vel = [ind_vel (12:14)+(i-1)*21];
    ind_omega = [ind_omega (15:17)+(i-1)*21];
    ind_l_vel = [ind_l_vel (18)+(i-1)*21];
    ind_wheel_omega = [ind_wheel_omega (19:21)+(i-1)*21];
end
mpc_ind = find(wasMPCupdated(1:end-N)>0);

tau = [];
tau_u = [];
for i = 1:size(mpc_ind,1)
    tau = [tau elapsed_time(mpc_ind(i),:)'];
    tau_u = [tau_u elapsed_time(mpc_ind(i),1:end-1)'];
end

tau = tau-0.002;
tau_u = tau_u-0.002;


% mpc_ind(1) = 0;
% mpc_ind = mpc_ind+1;

% tau = tau(:,mpc_ind)-2*dt;
% tau_u = tau_u(:,mpc_ind)-2*dt;

pos_MPC = sol(mpc_ind,ind_pos);
quat_MPC = sol(mpc_ind,ind_quat);
tmp = quat_MPC;
quat_MPC(:,1:4:end) = tmp(:,4:4:end);
quat_MPC(:,2:4:end) = tmp(:,1:4:end);
quat_MPC(:,3:4:end) = tmp(:,2:4:end);
quat_MPC(:,4:4:end) = tmp(:,3:4:end);
l_MPC = sol(mpc_ind,ind_l);
wheel_MPC = sol(mpc_ind,ind_w);
vel_MPC = sol(mpc_ind,ind_vel);
omega_MPC = sol(mpc_ind,ind_omega);
l_vel_MPC = sol(mpc_ind,ind_l_vel);
wheel_omega_MPC = sol(mpc_ind,ind_wheel_omega);
torque_MPC = sol(mpc_ind,21*N+1:end);
d_MPC = d_bar(mpc_ind,:);
elapsed_mpc = elapsed_time(mpc_ind,:);



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
tabPlotSingle(tg,t-t_start,dt,pos,'Pos')
% tabPlot(tg,t,dt,ph,pos,pos_pred,'Pos')
for i =1:3
    tmp_y = [pos_MPC(:,i:3:end)'; nan(1,size(tau,2))];
    tmp_x = [tau-t_start;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
%     scatter(tau(1,:), pos_MPC(:,i),repmat(30,size(tau,2),1),repmat(c3(i,:),size(tau,2),1),'filled')
end
if sim
plot(t-t_start,command_d(:,1),'--','linewidth',5,'color',c3(1,:));
plot(t-t_start,command_d(:,2),'--','linewidth',5,'color',c3(2,:));
end
% axis([1 3 -0.8,0.9])
% set(gca,'FontSize',30)
% set(gca,'TickLabelInterpreter','latex')
% xlabel('Time','interpreter','latex')
% ylabel('Position [m]','interpreter','latex')

% plot(t,contact)
tabPlotSingle(tg,t,dt,quat,'Quat')
for i =1:4
    tmp_y = [quat_MPC(:,i:4:end)'; nan(1,size(tau,2))];
    tmp_x = [tau;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c4(i,:))
%     scatter(tau(1,:), quat_MPC(:,i),repmat(30,size(tau,2),1),repmat(c4(i,:),size(tau,2),1),'filled')
end
tabPlotSingle(tg,t-t_start,dt,vel,'Vel')
for i =1:3
    tmp_y = [vel_MPC(:,i:3:end)'; nan(1,size(tau,2))];
    tmp_x = [tau-t_start;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
%     scatter(tau(1,:), vel_MPC(:,i),repmat(30,size(tau,2),1),repmat(c3(i,:),size(tau,2),1),'filled')
end
% axis([1 3 -4 4])
% set(gca,'FontSize',30)
% set(gca,'TickLabelInterpreter','latex')
% xlabel('Time','interpreter','latex')
% ylabel('Position [m]','interpreter','latex')

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

tabPlotSingle(tg,t,dt,quat*0,'d_bar')
tmp_y = [d_MPC(:,1:end)'; nan(1,size(tau,2))];
tmp_x = [tau(1:end-1,:);  nan(1,size(tau,2))];
plot(tmp_x(:), tmp_y(:),'--','color',c3(1,:))

tabPlotSingle(tg,t,dt,omega,'Omega')
for i =1:3
    tmp_y = [omega_MPC(:,i:3:end)'; nan(1,size(tau,2))];
    tmp_x = [tau;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
end
tabPlotSingle(tg,t,dt,l,'Length')
tmp_y = [l_MPC(:,1:end)'; nan(1,size(tau,2))];
tmp_x = [tau;  nan(1,size(tau,2))];
plot(tmp_x(:), tmp_y(:),'--','color',c3(1,:))
tabPlotSingle(tg,t,dt,torque,'Torque')
for i =1:3
    tmp_y = [torque_MPC(:,(i+1):4:end)'; nan(1,size(tau_u,2))];
    tmp_x = [tau_u;  nan(1,size(tau_u,2))];
    stairs(tmp_x(:), tmp_y(:),'.-','color',c4(i+1,:))
end
tabPlotSingle(tg,t,dt,wheel_vel,'Wheel Vel')
for i =1:3
    tmp_y = [wheel_omega_MPC(:,i:3:end)'; nan(1,size(tau,2))];
    tmp_x = [tau;  nan(1,size(tau,2))];
    plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
end


%% 
fig_pos = [0 0 1000 1000];

%% Vel Close
% figure(1);
% set(gcf,'renderer','painters');
% set(gcf,'Position',fig_pos)
% clf;
% hold on;
% plot(t-t_start,vel(:,3),'linewidth',3,'color',c3(3,:))
% plot(t-t_start,vel(:,2),'linewidth',3,'color',c3(2,:))
% plot(t-t_start,vel(:,1),'linewidth',3,'color',c3(1,:))
% for i =3:-1:1
% tmp_y = [vel_MPC(:,i:3:end)'; nan(1,size(tau,2))];
% tmp_x = [tau-t_start;  nan(1,size(tau,2))];
% plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
% %     scatter(tau(1,:), vel_MPC(:,i),repmat(30,size(tau,2),1),repmat(c3(i,:),size(tau,2),1),'filled')
% end
% axis([1 3 -4 4])
% set(gca,'FontSize',30)
% set(gca,'TickLabelInterpreter','latex')
% xlabel('Time','interpreter','latex')
% ylabel('Velocity [m/s]','interpreter','latex')
% 
% % Pos Close
% figure(2);
% set(gcf,'renderer','painters');
% set(gcf,'Position',fig_pos)
% clf;
% hold on;
% plot(t-t_start,pos(:,3),'linewidth',3,'color',c3(3,:))
% plot(t-t_start,pos(:,2),'linewidth',3,'color',c3(2,:))
% plot(t-t_start,pos(:,1),'linewidth',3,'color',c3(1,:))
% % tabPlot(tg,t,dt,ph,pos,pos_pred,'Pos')
% for i =3:-1:1
%     tmp_y = [pos_MPC(:,i:3:end)'; nan(1,size(tau,2))];
%     tmp_x = [tau-t_start;  nan(1,size(tau,2))];
%     plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
% %     scatter(tau(1,:), pos_MPC(:,i),repmat(30,size(tau,2),1),repmat(c3(i,:),size(tau,2),1),'filled')
% end
% if sim
% plot(t-t_start,command_d(:,1),'--','linewidth',5,'color',c3(1,:));
% plot(t-t_start,command_d(:,2),'--','linewidth',5,'color',c3(2,:));
% end
% axis([1 3 -0.8,0.9])
% set(gca,'FontSize',30)
% set(gca,'TickLabelInterpreter','latex')
% xlabel('Time','interpreter','latex')
% ylabel('Position [m]','interpreter','latex')
% 
% % Vel
% figure(3);
% set(gcf,'renderer','painters');
% set(gcf,'Position',fig_pos)
% clf;
% hold on;
% plot(t-t_start,vel(:,3),'linewidth',3,'color',c3(3,:))
% plot(t-t_start,vel(:,2),'linewidth',3,'color',c3(2,:))
% plot(t-t_start,vel(:,1),'linewidth',3,'color',c3(1,:))
% for i =3:-1:1
% tmp_y = [vel_MPC(:,i:3:end)'; nan(1,size(tau,2))];
% tmp_x = [tau-t_start;  nan(1,size(tau,2))];
% plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
% %     scatter(tau(1,:), vel_MPC(:,i),repmat(30,size(tau,2),1),repmat(c3(i,:),size(tau,2),1),'filled')
% end
% axis([0 15 -4 4])
% set(gca,'FontSize',30)
% set(gca,'TickLabelInterpreter','latex')
% xlabel('Time','interpreter','latex')
% ylabel(['Velocity [m/s]' ...
%     ''],'interpreter','latex')
% 
% % Pos
% figure(4);
% set(gcf,'renderer','painters');
% set(gcf,'Position',fig_pos)
% clf;
% hold on;
% plot(t-t_start,pos(:,3),'linewidth',3,'color',c3(3,:))
% plot(t-t_start,pos(:,2),'linewidth',3,'color',c3(2,:))
% plot(t-t_start,pos(:,1),'linewidth',3,'color',c3(1,:))
% % tabPlot(tg,t,dt,ph,pos,pos_pred,'Pos')
% for i =3:-1:1
%     tmp_y = [pos_MPC(:,i:3:end)'; nan(1,size(tau,2))];
%     tmp_x = [tau-t_start;  nan(1,size(tau,2))];
%     plot(tmp_x(:), tmp_y(:),'--','color',c3(i,:))
% %     scatter(tau(1,:), pos_MPC(:,i),repmat(30,size(tau,2),1),repmat(c3(i,:),size(tau,2),1),'filled')
% end
% if sim
% plot(t-t_start,command_d(:,1),'--','linewidth',5,'color',c3(1,:));
% plot(t-t_start,command_d(:,2),'--','linewidth',5,'color',c3(2,:));
% end
% axis([0 15 -1.1, 1.1])
% set(gca,'FontSize',30)
% set(gca,'TickLabelInterpreter','latex')
% xlabel('Time','interpreter','latex')
% ylabel('Position [m]','interpreter','latex')


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
    plot(t,x(:,i),'color',c(i,:),'linewidth',3);
end
% for tau = 0:dt:t(end)
%     line([tau tau],ax.YLim,'color',[0 0 0 .1])
% %     xline(tau,'alpha',.1)
% end
end
