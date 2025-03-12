clear; clc;

% d = readtable("/media/noelcs/Bia/foot_data.txt");
d = readtable("../data/data_hardware.csv");
%%
figure(1)
clf;
subplot(2,2,1)
plot(d.t, d.legpos)
ylim([0 0.05])
ylabel('Foot Pos')
subplot(2,2,2)
plot(d.t, d.legvel)
ylim([-1 1])
ylabel('Foot Vel')

subplot(2,2,3)
plot(d.t, d.contact)
ylabel('Contact')

% subplot(2,2,1)
% plot(d{:, 1}, d{:, 7})
% ylabel('Motor Vel')T_post_log
