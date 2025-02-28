clear;clc;

% d_ns = readtable("../data/data_hardware_no_standoff.csv");
% d_ws = readtable("../data/data_hardware_standoff.csv");
d = readtable("../data/data_hardware.csv");

%%

yaw_0_opti = quat2yaw([d.optitrackqw(1), d.optitrackqx(1), d.optitrackqy(1), d.optitrackqz(1)]);
yaw_0 = quat2yaw([d.qw(1), d.qx(1), d.qy(1), d.qz(1)]);

yaw_off = 0;
Ryaw = [
    cos(yaw_off), sin(yaw_off);
    -sin(yaw_off), cos(yaw_off)
    ];

tmp_pos = (Ryaw * [d.optitrackx d.optitracky]')';
o_x = tmp_pos(:, 1);
o_y = tmp_pos(:, 2);

tmp_vel = (Ryaw * [d.optitrackxdot d.optitrackydot]')';
o_x_global = tmp_vel(:, 1);
o_y_global = tmp_vel(:, 2);

t_range = [2, 100];

figure(1)
clf;
subplot(2,2,1)
hold on
plot(d.t, d.xdot)
plot(d.t, o_x_global)
legend('t265', 'optitrack')
ylabel('Vx')
xlim(t_range)

subplot(2,2,2)
hold on
plot(d.t, d.ydot)
plot(d.t, o_y_global)
legend('t265', 'optitrack')
ylabel('Vy')
xlim(t_range)

subplot(2,2,3)
hold on
plot(d.t, d.zdot)
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



%% Time to filter

vx = d.xdot;
vy = d.ydot;
vz = d.zdot;

yaw_0_opti = quat2yaw([d.optitrackqw(1), d.optitrackqx(1), d.optitrackqy(1), d.optitrackqz(1)]);
yaw_0 = quat2yaw([d.qw(1), d.qx(1), d.qy(1), d.qz(1)]);

yaw_off = 0;
Ryaw = [
    cos(yaw_off), sin(yaw_off);
    -sin(yaw_off), cos(yaw_off)
    ];

tmp_pos = (Ryaw * [d.optitrackx d.optitracky]')';
x_opt = tmp_pos(:, 1);
y_opt = tmp_pos(:, 2);

tmp_vel = (Ryaw * [d.optitrackxdot d.optitrackydot]')';
vx_opt = tmp_vel(:, 1);
vy_opt = tmp_vel(:, 2);

window_len = 100;
vx_opt_filter = conv(vx_opt, ones(1, window_len)/window_len, 'same');
vy_opt_filter = conv(vy_opt, ones(1, window_len)/window_len, 'same');

figure(1)
clf
subplot(2,2,1)
hold on
plot(d.t, vx_opt)
plot(d.t, vx_opt_filter)
legend('v_x Opti', 'v_x Opti filtered')

subplot(2,2,2)
hold on
plot(d.t, vy_opt)
plot(d.t, vy_opt_filter)
legend('v_y Opti', 'v_y Opti filtered')

vx_filt = zeros(size(vx));
vy_filt = zeros(size(vy));
vz_filt = zeros(size(vz));

vx_running = vx(1);
vy_running = vy(1);
vz_running = vz(1);

alpha = 0.95;

for i = 1:size(d.t, 1)
    vx_running = alpha * vx_running + (1 - alpha) * vx(i);
    vy_running = alpha * vy_running + (1 - alpha) * vy(i);
    vz_running = alpha * vz_running + (1 - alpha) * vz(i);

    vx_filt(i) = vx_running;
    vy_filt(i) = vy_running;
    vz_filt(i) = vz_running;
end

subplot(2,2,3)
hold on
plot(d.t, vx)
plot(d.t, vx_filt)
plot(d.t, vx_opt_filter)
legend('v_x', 'v_x filt', 'v_x Opti filtered')

subplot(2,2,4)
hold on
plot(d.t, vy)
plot(d.t, vy_filt)
plot(d.t, vy_opt_filter)
legend('v_y', 'v_y filt', 'v_y Opti filtered')

%%
figure(1)
clf
subplot(2,1,1)
hold on
plot(d.t, d.camxdot)
plot(d.t, d.camxdotfilt)
legend('cam xdot', 'cam xdot filtered')

subplot(2,1,2)
hold on
plot(d.t, d.camydot)
plot(d.t, d.camydotfilt)
legend('cam ydot', 'cam ydot filtered')


wx_filt = zeros(size(d.wx));
wy_filt = zeros(size(d.wy));
wx_running = d.wx(1);
wy_running = d.wy(1);

alpha = 0.9;

for i = 1:size(d.t, 1)
    wx_running = alpha * wx_running + (1 - alpha) * d.wx(i);
    wy_running = alpha * wy_running + (1 - alpha) * d.wy(i);

    wx_filt(i) = wx_running;
    wy_filt(i) = wy_running;
end

B = 1/10*ones(10,1);
wx_wind_filt = filter(B,1, d.wx);
wy_wind_filt = filter(B,1, d.wy);


figure(2)
clf
subplot(2,1,1)
hold on
plot(d.t, d.wx)
plot(d.t, wx_filt)
plot(d.t, wx_wind_filt)
legend('wx', 'wx filt', 'wx wind')

subplot(2,1,2)
hold on
plot(d.t, d.wy)
plot(d.t, wy_filt)
plot(d.t, wy_wind_filt)
legend('wy', 'wy filt', 'wy wind')

%% Helper

function yaw = quat2yaw(quat)
qw = quat(1);
qx = quat(2);
qy = quat(3);
qz = quat(4);
yaw = atan2(2 * (qw * qz+ qx * qy), 1 - 2 * (qy^2 + qz^2));

end