clear; clc;

targ_file = '../data/data_foot.txt';
copyfile("/media/noelcs/94B7-16DE/foot_data.txt", targ_file);
d = readtable(targ_file);
c = strcmp(d{:, 2}, "Compression");
r = strcmp(d{:, 2}, "Release");
t = d{:, 1} - d{1, 1};

%%
t_span = [0, 13];
figure(1)
clf
subplot(3,2,1)
hold on
plot(t, d{:, 4})
xlim(t_span)
title('Foot Pos')

subplot(3,2,2)
plot(t, d{:, 5})
title('Foot Vel')
xlim(t_span)

subplot(3,2,3)
plot(t, d{:, 6})
title('Motor Pos')
xlim(t_span)

subplot(3,2,4)
plot(t, d{:, 7})
title('Motor Vel')
xlim(t_span)

subplot(3,2,5)
plot(t, d{:, end})
title("Torque")
xlim(t_span)

subplot(3,2,6)
hold on
plot(t, d{:, 3})
plot(t, c * 0.8)
plot(t, r * 0.4)
legend('fsm', 'compression', 'release')
title('fsm')


