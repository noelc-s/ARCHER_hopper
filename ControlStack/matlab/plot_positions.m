clear; clc; 

d = readtable('../data/data_hardware.csv');

%%
figure(1)
clf
patch([d.x; NaN], [d.y; NaN], [d.t; NaN],'EdgeColor','interp','FaceColor','none','LineWidth',2)
colorbar


q = [d.q_w, d.q_x, d.q_y, d.q_z];
cam_q = [d.cam_qw, d.cam_qx, d.cam_qy, d.cam_qz];

imu_yaw = quat2yaw(q);
cam_yaw = quat2yaw(cam_q);

figure(2)
clf;
subplot(2,1,1)
hold on
plot(d.t, imu_yaw)
plot(d.t, cam_yaw, '--')
plot(d.t, d.vn_yaw)
legend('Control yaw', 'Cam yaw', 'IMU yaw (removed)')

subplot(2,1,2)
hold on
plot(d.t, d.w_3)
plot(d.t, ones(size(d.t)) * mean(d.w_3))

figure(3)
clf
hold on
plot(d.t, q)
set(gca, 'ColorOrderIndex', 1)
plot(d.t, cam_q, '--')


function yaw = quat2yaw(quat)
qw = quat(:, 1);
qx = quat(:, 2);
qy = quat(:, 3);
qz = quat(:, 4);
yaw = atan2(2 * (qw .* qz + qx .* qy), 1 - 2 * (qy.^2 + qz.^2));
end