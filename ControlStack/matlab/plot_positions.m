clear; clc;

% d = readtable('../data/data_hardware.csv');
d = readtable('../data/data_orin.csv');

%%
figure(1)
clf
patch([d.x; NaN], [d.y; NaN], [d.t; NaN],'EdgeColor','interp','FaceColor','none','LineWidth',2)
colorbar


q = [d.qw, d.qx, d.qy, d.qz];
cam_q = [d.camqw, d.camqx, d.camqy, d.camqz];
q_des = [d.qwdes, d.qxdes, d.qydes, d.qzdes];

q(q(:, 1) < 0) = q(q(:, 1) < 0) * -1;
q_des(q_des(:, 1) < 0) = q_des(q_des(:, 1) < 0) * -1;

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
plot(d.t, d.wz)
plot(d.t, d.wzunfilt)
plot(d.t, ones(size(d.t)) * mean(d.wz))

figure(3)
clf
hold on
plot(d.t, q)
set(gca, 'ColorOrderIndex', 1)
plot(d.t, q_des, '--')



%%
close all
figure(4);
clf

T = mean(diff(d.t));  % Sampling period (seconds)
Fs = 1 / T;

L = size(d.t, 1);  % Length of the signal
if mod(L, 2) == 1
    L = L - 1;
end
t = d.t(1:L); % Time vector
for i = 1:3
    
    % Compute FFT
    if i == 1
        signal = d.wx(1:L);
    elseif i == 2
        signal = d.wy(1:L);
    else
        signal = d.wz(1:L);
    end
    Y = fft(signal);
    P2 = abs(Y/L);        % Two-sided spectrum
    P1 = P2(1:L/2+1);     % Single-sided spectrum
    P1(2:end-1) = 2*P1(2:end-1); % Scale amplitude correctly

    % Frequency axis
    f = Fs * (0:(L/2)) / L;

    % Plot FFT
    
    subplot(3,2,2 * (i - 1) + 1)
    plot(t, Y)
    hold on
    plot(t, 50 * sin(53 * 2 * pi * t))
    xlim([50, 50 + 10 / 53])
    subplot(3,2, 2 * (i - 1) + 2)
    plot(f, P1);
    title('Single-Sided Amplitude Spectrum of Signal');
    xlabel('Frequency (Hz)');
    ylabel('|P1(f)|');
    grid on;
    % xlim([0, 125])
end


%%
function yaw = quat2yaw(quat)
qw = quat(:, 1);
qx = quat(:, 2);
qy = quat(:, 3);
qz = quat(:, 4);
yaw = atan2(2 * (qw .* qz + qx .* qy), 1 - 2 * (qy.^2 + qz.^2));
end