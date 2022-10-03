k = 1000;

kp = 200;
kd = 2*sqrt(kp);

x_d = 1;
u = @(x)  -kp*(x(1) - x_d) - kd*x(2) + 10; %  + k*x(1)

[t,x] = ode45(@(t,x) [x(2);-k*x(1) + u(x)], [0 1], [0 0])

plot(t,x)
