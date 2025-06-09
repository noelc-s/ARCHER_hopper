system('sshpass -p "amberlab" scp noelcs@192.168.1.2:~/ARCHER_hopper/ControlStack/data/data_hardware.csv ~/repos/ARCHER_hopper/ControlStack/data/data.csv')

A = readtable('../data/data.csv');

%%

clf
plot(A.x, A.y)
hold on
plot(A.cmd1, A.cmd2)
plot(A.graph_plan_x, A.graph_plan_y,'o--')

legend({'path', 'joystick','graph solve'})

%%

clf

subplot(2,1,1)
plot(A.t,A.x)
hold on
plot(A.t,A.graph_plan_x,'o')
plot(A.t,A.cmd1,'o')

subplot(2,1,2)
plot(A.t,A.y)
hold on
plot(A.t,A.graph_plan_y,'o')
plot(A.t,A.cmd2,'o')

%%

clf

plot(A.x, A.y)
hold on
plot(A.graph_plan_x,A.graph_plan_y,'ro-')
plot(A.cmd1,A.cmd2,'ko-')


% why was there a yaw des before? Log is weird
% now gains are not tuned.
% no explanation for foot

%%
q = [A.qw, A.qx, A.qy, A.qz];
qdes = [A.qwdes, A.qxdes, A.qydes, A.qzdes];
lims = [0, A.t(end) - 1];
% lims = [24, 34];

clf
[rolldes, pitchdes, yawdes] = quat2angle(qdes, 'XYZ');
[roll, pitch, yaw] = quat2angle(q, 'XYZ');
subplot(2,2,1)
hold on
plot(A.t, [roll, pitch yaw]) 
set(gca, 'ColorOrderIndex', 1)
plot(A.t, [rolldes, pitchdes yawdes], '--') 
plot(A.t, A.contact)
xlim(lims)
legend('roll', 'pitch','yaw', 'rolldes', 'pitchdes','yawdes')
% subplot(2,2,1)
% hold on
% plot(A.t, q) 
% plot(A.t, qdes) 
% plot(A.t, A.contact)
% xlim(lims)
% legend('w', 'x', 'y', 'z')

subplot(2,2,3)
hold on
plot(A.t, [A.xdot, A.ydot, A.zdot])
plot(A.t, A.contact)
legend('vx', 'vy', 'vz')
xlim(lims)


% subplot(2,2,2)
% hold on
% pitch_des = 0.2 * min(max(A.graph_plan_x - A.x, -0.15), 0.15) + 0.3 * min(max(-A.xdot, -1), 1) + 0.03;
% plot(A.t, pitch_des)
% [~, pdes, ~] = quat2angle(qdes);
% plot(A.t, pdes)
% legend('pitch des pd', 'pitch des quatdes')
% xlim(lims)

subplot(2,2,2)
hold on
plot(A.t, A.y)
plot(A.t, A.graph_plan_y)
plot(A.t, A.graph_center_y)
legend('y', 'ydes')
xlim(lims)

subplot(2,2,4)
hold on
plot(A.t, A.x)
plot(A.t, A.graph_plan_x)
plot(A.t, A.graph_center_x)
legend('x', 'xdes')
xlim(lims)

%%
col = find(strcmp(fieldnames(A), 'graphsol0') == 1);
x = A(:,col:2:end);
y = A(:,(col+1):2:end);

while(1)
plan_path = [];
hopper_path = [];
for i = 1:100:size(x,1)
    tic;
    clf
    hold on
    if i > 1
        plot(x{1:i,1}, y{1:i,1}, 'k.-')
        plot(A.x(1:i), A.y(1:i), 'b.-')
    end
    plot(x{i,:},y{i,:},'ro-')
    scatter(A.cmd1(i),A.cmd2(i),200,'filled');
    title(sprintf("Time: %0.2f", i / 1000)) 

    xlim([-2.5, 2.5])
    ylim([-2.5, 2.5])
    drawnow
    pause(0.1 - (toc))
end
pause
end

%%
q = [A.qw, A.qx, A.qy, A.qz];
qdes = [A.qwdes, A.qxdes, A.qydes, A.qzdes];
lims = [0, A.t(end) - 1];
% lims = [24, 34];

clf
[rolldes, pitchdes, yawdes] = quat2angle(qdes, 'XYZ');
[roll, pitch, yaw] = quat2angle(q, 'XYZ');
subplot(3,1,1)
plot(A.t, A.x)
subplot(3,1,2)
plot(A.t, A.y)
subplot(3,1,3)
plot(A.t, yaw)
