clear;clc;

d = readtable("data_hardware.csv");

%%
yaw_0_opti = quat2yaw([d.optitrackqw(1), d.optitrackqx(1), d.optitrackqy(1), d.optitrackqz(1)]);
yaw_0 = quat2yaw([d.qw(1), d.qx(1), d.qy(1), d.qz(1)]);

yaw_off = -1.02;
theta = yaw_0_opti - yaw_0 - yaw_off;
Ryaw = [
    cos(theta), sin(theta);
    -sin(theta), cos(theta)
];

tmp_pos = (Ryaw * [d.optitrackx d.optitracky]')';
o_x = tmp_pos(:, 1);
o_y = tmp_pos(:, 2);

tmp_vel = (Ryaw * [d.optitrackxdot d.optitrackydot]')';
o_x_global = tmp_vel(:, 1);
o_y_global = tmp_vel(:, 2);

t_range = [2, 10];

figure(1)
clf;
subplot(2,2,1)
hold on
plot(d.t, d.globalxdot)
plot(d.t, o_x_global)
legend('t265', 'optitrack')
ylabel('Vx')
xlim(t_range)

subplot(2,2,2)
hold on
plot(d.t, d.globalydot)
plot(d.t, o_y_global)
legend('t265', 'optitrack')
ylabel('Vy')
xlim(t_range)

subplot(2,2,3)
hold on
plot(d.t, d.globalzdot)
plot(d.t, d.optitrackzdot)
ylabel('Vz')
legend('t265', 'optitrack')
xlim(t_range)

figure(2)
clf;
subplot(2,2,1)
hold on
plot(d.t, d.x - d.x(1))
plot(d.t, o_x - o_x(1))
ylabel('X')
legend('t265', 'optitrack')
xlim(t_range)

subplot(2,2,2)
hold on
plot(d.t, d.y - d.y(1))
plot(d.t, o_y - o_y(1))
legend('t265', 'optitrack')
ylabel('Y')
xlim(t_range)

subplot(2,2,3)
hold on
plot(d.t, d.z - d.z(1))
plot(d.t, d.optitrackz - d.optitrackz(1))
legend('t265', 'optitrack')
ylabel('Z')
xlim(t_range)

% 
figure(3)
clf;
subplot(2,2,1)
hold on
plot(d.t, d.wx)
legend('t265')
ylabel('Vx')
xlim(t_range)

subplot(2,2,2)
hold on
plot(d.t, d.wy)
legend('t265')
ylabel('Vy')
xlim(t_range)

subplot(2,2,3)
hold on
plot(d.t, d.wz)
ylabel('Vz')
legend('t265')
xlim(t_range)

subplot(2,2,4)
hold on
plot(d.t, d.globalzdot)
plot(d.t, d.optitrackzdot)
ylabel('Vz')
legend('t265', 'optitrack')
xlim(t_range)

figure(5)
clf;
subplot(2,2,1)
hold on
plot(d.t, d.globalxdot)
plot(d.t, o_x_global)
legend('t265', 'optitrack')
ylabel('Vx')
xlim(t_range)

subplot(2,2,2)
hold on
plot(d.t, d.wy)
ylabel('Wy')
xlim(t_range)

subplot(2,2,3)
hold on
plot(d.t, d.globalzdot)
plot(d.t, d.optitrackzdot)
ylabel('Vz')
legend('t265', 'optitrack')
xlim(t_range)

subplot(2,2,4)
hold on
plot(d.t, d.wz)
ylabel('Wz')
xlim(t_range)

%% 


t_range = [11 11.5];
inds = d.t > t_range(1) & d.t < t_range(2);

order = 3; framelen = 25;
% x_filt = sgolayfilt(o_x_global,order,framelen);
% y_filt = sgolayfilt(o_y_global,order,framelen);
% z_filt = sgolayfilt(d.optitrackzdot,order,framelen);
x_filt = o_x_global;
y_filt = o_y_global;
z_filt = d.optitrackzdot;

r_c2b_d = [-0.067, -0.05, 0.121];
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');

% loss = @(r) norm([x_filt(inds) y_filt(inds) z_filt(inds)] - [d.camxdot(inds) + (d.camwy(inds) * r(3) - d.camwz(inds) * (r_c2b_d(2) + r(2))),...
%              d.camydot(inds) + (d.camwz(inds) * r(1) - d.camwx(inds) * r(3)),...
%              d.camzdot(inds) + (d.camwx(inds) * r(22) - d.camwy(inds) * r(1))],2);
% [r_cam_to_body,fval] = fmincon(loss, [0 0 0],[],[],[],[],[],[],[],options);

loss2 = @(r) sum(abs([x_filt(inds) y_filt(inds) z_filt(inds)] - [d.camxdot(inds) + (d.camwy(inds) * (r_c2b_d(3) + r(3)) - d.camwz(inds) * r(2)),...
             d.camydot(inds) + (d.camwz(inds) * (r_c2b_d(1) + r(1)) - d.camwx(inds) * (r_c2b_d(3) + r(3))),...
             d.camzdot(inds) + (d.camwx(inds) * (r_c2b_d(2) + r(2)) - d.camwy(inds) * (r_c2b_d(1) + r(1)))]), 'all');


[dr,fval] = fmincon(loss2, [0 0 0],[],[],[],[],[],[],[],options);
r_cam_to_body = r_c2b_d + dr;

r_cam_to_body = [-0.04 -0.05, 0.14];
fprintf("Default:\n")
disp(r_c2b_d)
fprintf("Optimized: \n")
disp(r_cam_to_body)

body_vel =  [d.camxdot + (d.camwy * r_cam_to_body(3) - d.camwz * r_cam_to_body(2)),...
             d.camydot + (d.camwz * r_cam_to_body(1) - d.camwx * r_cam_to_body(3)),...
             d.camzdot + (d.camwx * r_cam_to_body(2) - d.camwy * r_cam_to_body(1))];

% [X,Y] = meshgrid(linspace(-0.5,0.5));
% Z = X;
% for i=1:numel(Z)
%     i
%     Z(i) = loss(r_cam_to_body + [X(i) Y(i) 0]);
% end


figure(4)
clf;

subplot(3,1,1)
plot(d.t, body_vel(:,1));
hold on
plot(d.t, d.xdot);
plot(d.t, x_filt);
xlim(t_range)

subplot(3,1,2)
plot(d.t, body_vel(:,2));
hold on
plot(d.t, d.ydot);
plot(d.t, y_filt);
xlim(t_range)

subplot(3,1,3)
plot(d.t, body_vel(:,3));
hold on
plot(d.t, d.zdot);
plot(d.t, z_filt);
xlim(t_range)

% eqn: opti_vel = cam_vel + R * cam_W


function yaw = quat2yaw(quat)
qw = quat(1);
qx = quat(2);
qy = quat(3);
qz = quat(4);
yaw = atan2(2 * (qw * qz+ qx * qy), 1 - 2 * (qy^2 + qz^2));

end