%% Read Data
clear;clc;%clf;

directory = '../data/Saved Data/';
files = dir(fullfile(directory, '*.csv'));

num_files = numel(files);
data = cell(num_files, 1);  % Preallocate cell array to store data

for i = 1:num_files
    filename = fullfile(directory, files(i).name);
    data{i} = readmatrix(filename);
end

P_E = [];
V_E = [];
E_D = [];
E_A = [];
%%
for j = 1:length(data)
    j
    A = data{j};

a = [1,0,0]';
b = [0,1,0]';
c = [0,0,1]';


% dt = param.Debug.dt;
dt = 0.01;
% MPC_dt = param.MPC.dt;
% ph = param.Debug.predHorizon;

ind = 1;
t = (A(:,ind)-A(1,ind));        ind=ind+1; % Sample time
contact = A(:,ind);             ind=ind+1;
optitrack_logged = A(:,ind);    ind=ind+1;
pos = A(:,ind:ind+2);           ind=ind+3;
leg_pos = A(:,ind);           ind=ind+1;
vel = A(:,ind:ind+2);           ind=ind+3;
leg_vel = A(:,ind);           ind=ind+1;
IMU_quat = A(:,ind:ind+3);          ind=ind+4;
tmp = IMU_quat; % .coeffs gives x, y, z, w
IMU_quat(:,1) = tmp(:,4); % w
IMU_quat(:,2) = tmp(:,1); % x
IMU_quat(:,3) = tmp(:,2); % y
IMU_quat(:,4) = tmp(:,3); % z
quat = A(:,ind:ind+3);          ind=ind+4;
tmp = quat; % .coeffs gives x, y, z, w
quat(:,1) = tmp(:,4); % w
quat(:,2) = tmp(:,1); % x
quat(:,3) = tmp(:,2); % y
quat(:,4) = tmp(:,3); % z
quat_d = A(:,ind:ind+3);          ind=ind+4;
tmp = quat_d; % .coeffs gives x, y, z, w
quat_d(:,1) = tmp(:,4); % w
quat_d(:,2) = tmp(:,1); % x
quat_d(:,3) = tmp(:,2); % y
quat_d(:,4) = tmp(:,3); % z
omega = A(:,ind:ind+2);         ind=ind+3;
torque = A(:,ind:ind+3);        ind=ind+4;
error = A(:,ind:ind+2);         ind=ind+3;
wheel_vel = A(:,ind:ind+2);         ind=ind+3;
try
command = A(:,ind:ind+2);       ind=ind+3;
catch E
    disp("No logged command");
    continue % did not log command for a few
end

for i = 1:length(t)
    q_a = quat(i,:);
    q_d = IMU_quat(i,:);
    err(i,:) = quat2eul(quat_prod(quat_inv(q_d),q_a));
end

% eul
for i = 1:length(t)
    q_a = quat(i,:);
    q_d = quat_d(i,:);
    eul_a(i,:) = quat2eul(q_a);
    eul_d(i,:) = quat2eul(q_d);
end


%
[~,ind] = findpeaks(contact);
ind = ind-1; % get pre-impact index;
P_E = [P_E; pos(ind,1:2) - command(ind,1:2)];
V_E = [V_E; vel(ind,1:2)];
E_A = [E_A; eul_a(ind,2:3)];
E_D = [E_D; eul_d(ind,2:3)];
end

%%
figure(1)
clf
scatter3(P_E(:,1), V_E(:,1),E_D(:,1),100,[0.8 0.070 0.010],'filled')
hold on
scatter3(P_E(:,1), V_E(:,1),E_A(:,1),100,[0.10 0.10 0.980],'filled')
axis([ -2 2 -0.5 0.5 -0.12 0.12])
% figure(2)
% scatter3(P_E(:,2), V_E(:,2),E_D(:,2),'filled')
% hold on
% scatter3(P_E(:,2), V_E(:,2),E_A(:,2),'filled')

K = -[0.1901  0.2726];
x = [-1 1]*2;
y = [-0.5 0.5]*3;
[X, Y] = meshgrid(x, y);
Z = K(1) * X + K(2) * Y;
surf(X, Y, Z,'facealpha',0.2,'facecolor',[0.9290 0.6940 0.1250])
view(15,50)
legend({'$\psi(\mathbf{z}_{k+1})$',...
    '$\eta_{k+1}$',...
    'LQR Surface'},'interpreter','latex')
xlabel('$\mathbf z_1$','interpreter','latex')
ylabel('$\mathbf z_2$','interpreter','latex')
set(gca,'TickLabelInterpreter', 'latex');
set(gca,'FontSize',60)
set(gca,'linewidth',2)
set(gcf,'renderer','painters')

%%
function q_new = quat_prod(q1, q2)
q_new = [q1(1)*q2(1) - dot(q1(2:4), q2(2:4))...
    q1(1)*q2(2:4)+q2(1)*q1(2:4)+cross(q1(2:4),q2(2:4))];
end

function q_inv = quat_inv(q)
q_inv = [q(1) -q(2:4)];
end

function [q,l] = plot_quat(p, quat,color,frame)
q = [];
l = [];
for i = 1:size(frame,2)
    rP = quat_prod(quat_prod(quat, [0 frame(:,i)']), quat_inv(quat));
    rP = rP(2:4);

    q = [q plot3(p(1) + rP(1),p(2) + rP(2),p(3) + rP(3),color)];
    l = [l plot3([p(1);p(1) + rP(1)],[p(2);p(2) + rP(2)],[p(3);p(3) + rP(3)],'k')];
end
end