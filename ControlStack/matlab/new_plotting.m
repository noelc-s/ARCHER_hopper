clear;

d = readtable('../data/data_hardware.csv');

%% Positions
q = [d.qw d.qx d.qy d.qz];
camq = [d.camqw d.camqx d.camqy d.camqz];
yaw = quat2yaw(q);
camyaw = quat2yaw(camq);

figure(1)
clf;
subplot(2,2,1)
plot(d.t, yaw)
legend('yaw')
subplot(2,2,2)
plot(d.t, yaw)
legend('yaw')
subplot(2,2,3)
hold on
plot(d.t, d.x)
plot(d.t, d.y)
legend('global x', 'global y')

subplot(2,2,4)
hold on
plot(d.t, d.vx)
plot(d.t, d.vy)
legend('body vx', 'body vy')

function yaw = quat2yaw(quat)
qw = quat(:, 1);
qx = quat(:, 2);
qy = quat(:, 3);
qz = quat(:, 4);
yaw = atan2(2 * (qw .* qz+ qx .* qy), 1 - 2 * (qy.^2 + qz.^2));

end