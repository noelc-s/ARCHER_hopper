%% Read Data
A = readmatrix('../data/prediction.csv');
param = yaml.loadFile("../config/gains.yaml");

ind = 1;
pos = A(:,ind:ind+2);           ind=ind+3;
quat = A(:,ind:ind+3);          ind=ind+4;
quat = [quat(:,2:4) quat(:,1)]; % x,y,z,w convention
l = A(:,ind);                   ind=ind+1;
wheel_pos = A(:,ind:ind+2);      ind=ind+3;
vel = A(:,ind:ind+2);           ind=ind+3;
omega = A(:,ind:ind+2);         ind=ind+3;
l_dot = A(:,ind);               ind=ind+1;
wheel_vel = A(:,ind:ind+2);     ind=ind+3;

dt = 1;

%%
clf

hfig1 = figure(1);
set(hfig1,'WindowStyle','normal');
tg = uitabgroup(hfig1);

t = 1:size(pos,1);

tabPlotSingle(tg,t,dt,pos,'Pos')
tabPlotSingle(tg,t,dt,quat,'Quat')
tabPlotSingle(tg,t,dt,vel,'Vel')
tabPlotSingle(tg,t,dt,omega,'Omega')
tabPlotSingle(tg,t,dt,l,'Length')
tabPlotSingle(tg,t,dt,wheel_vel,'Wheel Vel')


function tabPlot(tg,t,dt,x, x_pred,title)
h = uitab(tg, 'Title', title);
ax = axes('Parent', h);
hold on
c = lines(size(x,2));
for i = 1:size(x,2)
    plot(t,x(:,i),'color',c(i,:));
    plot(t+dt,x_pred(:,i),'--','color',c(i,:));
end
for tau = 0:dt:t(end)
    line([tau tau],ax.YLim,'color',[0 0 0 .1])
%     xline(tau,'alpha',.1)
end
end

function tabPlotSingle(tg,t,dt,x,title)
h = uitab(tg, 'Title', title);
ax = axes('Parent', h);
hold on
c = lines(size(x,2));
for i = 1:size(x,2)
    plot(t,x(:,i),'color',c(i,:));
end
for tau = 0:dt:t(end)
    line([tau tau],ax.YLim,'color',[0 0 0 .1])
%     xline(tau,'alpha',.1)
end
end
