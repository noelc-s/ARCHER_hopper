%% Read Data
clear;clc;%clf;
A = readmatrix('../data/data.csv');

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
%%
for i = 1:length(t)
    q_a = quat(i,:);
    q_d = IMU_quat(i,:);
    err(i,:) = quat2eul(quat_prod(quat_inv(q_d),q_a));
end


%%
clf

speed = 1;
tail_ind = 100;
t_start = 0;

ind = 1;
hold on;
grid on;
view(-50,30);
[q1,l1] = plot_quat(pos(ind,:), quat(ind,:),'ro',[a b c]);
[q2,l2] = plot_quat(pos(ind,:), quat_d(ind,:),'bo',[a b c]);
tail1 = plot3(0,0,0,'r','linewidth',2);
tail2 = plot3(0,0,0,'r','linewidth',2);
tail3 = plot3(0,0,0,'r','linewidth',2);
% tail1d = plot3(0,0,0,'b','linewidth',2);
% tail2d = plot3(0,0,0,'b','linewidth',2);
% tail3d = plot3(0,0,0,'b','linewidth',2);

swirl1 = 0.1*[zeros(length(0:0.25:3*pi/2),1) cos(0:0.25:3*pi/2)' sin(0:0.25:3*pi/2)'];
swirl2 = 0.1*[cos(0:0.25:3*pi/2)' zeros(length(0:0.25:3*pi/2),1) sin(0:0.25:3*pi/2)'];
swirl3 = 0.1*[cos(0:0.25:3*pi/2)' sin(0:0.25:3*pi/2)' zeros(length(0:0.25:3*pi/2),1)];
swril_p1 = plot3(swirl1(:,1),swirl1(:,2),swirl1(:,3));
swril_p2= plot3(swirl2(:,1),swirl2(:,2),swirl2(:,3));
swril_p3 = plot3(swirl3(:,1),swirl3(:,2),swirl3(:,3));

tic
while(1)
    time = toc*speed + t_start;
    ind = find(t>time,1);
    if isempty(ind)
        break
    end
    axis equal
    axis([pos(ind,1)-2 pos(ind,1)+2 pos(ind,2)-2 pos(ind,2)+2 0 2])
    delete(q1); delete(l1);
    delete(q2); delete(l2);
    [q1,l1] = plot_quat(pos(ind,:), quat(ind,:),'ro',[a b c]);
    [q2,l2] = plot_quat(pos(ind,:), quat_d(ind,:),'bo',[a b c]);
    start_ind = max(1,ind - tail_ind);
    for j = 1:size(swirl1,1)
        ps1(j,:) =  quat_prod(quat_prod(quat(ind,:), [0 swirl1(j,:)]+0.5*[0 a']), quat_inv(quat(ind,:)));
        ps2(j,:) =  quat_prod(quat_prod(quat(ind,:), [0 swirl2(j,:)]+0.5*[0 b']), quat_inv(quat(ind,:)));
        ps3(j,:) =  quat_prod(quat_prod(quat(ind,:), [0 swirl3(j,:)]+0.5*[0 c']), quat_inv(quat(ind,:)));
    end
    swril_p1.XData = ps1(:,2)+pos(ind,1);
    swril_p1.YData = ps1(:,3)+pos(ind,2);
    swril_p1.ZData = ps1(:,4)+pos(ind,3);
    swril_p1.LineWidth = 0.4*abs(omega(ind,1))+0.1;
    swril_p2.XData = ps2(:,2)+pos(ind,1);
    swril_p2.YData = ps2(:,3)+pos(ind,2);
    swril_p2.ZData = ps2(:,4)+pos(ind,3);
    swril_p2.LineWidth = 0.4*abs(omega(ind,2))+0.1;
    swril_p3.XData = ps3(:,2)+pos(ind,1);
    swril_p3.YData = ps3(:,3)+pos(ind,2);
    swril_p3.ZData = ps3(:,4)+pos(ind,3);
    swril_p3.LineWidth = 0.4*abs(omega(ind,3))+0.1;
    for j = start_ind:ind
        px = quat_prod(quat_prod(quat(j,:), [0 a']), quat_inv(quat(j,:)));
        py = quat_prod(quat_prod(quat(j,:), [0 b']), quat_inv(quat(j,:)));
        pz = quat_prod(quat_prod(quat(j,:), [0 c']), quat_inv(quat(j,:)));
        rP(j-start_ind+1,:) = [pos(j,:) + px(2:4) pos(j,:) + py(2:4) pos(j,:) + pz(2:4)];
        % pxd = quat_prod(quat_prod(quat_d(j,:), [0 a']), quat_inv(quat_d(j,:)));
        % pyd = quat_prod(quat_prod(quat_d(j,:), [0 b']), quat_inv(quat_d(j,:)));
        % pzd = quat_prod(quat_prod(quat_d(j,:), [0 c']), quat_inv(quat_d(j,:)));
        % rPd(j-start_ind+1,:) = [pos(j,:) + pxd(2:4) pos(j,:) + pyd(2:4) pos(j,:) + pzd(2:4)];
    end
    tail1.XData = rP(:,1);
    tail1.YData = rP(:,2);
    tail1.ZData = rP(:,3);
    tail2.XData = rP(:,4);
    tail2.YData = rP(:,5);
    tail2.ZData = rP(:,6);
    tail3.XData = rP(:,7);
    tail3.YData = rP(:,8);
    tail3.ZData = rP(:,9);
    % tail1d.XData = rPd(:,1);
    % tail1d.YData = rPd(:,2);
    % tail1d.ZData = rPd(:,3);
    % tail2d.XData = rPd(:,4);
    % tail2d.YData = rPd(:,5);
    % tail2d.ZData = rPd(:,6);
    % tail3d.XData = rPd(:,7);
    % tail3d.YData = rPd(:,8);
    % tail3d.ZData = rPd(:,9);

    drawnow;
    clear rP
end

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