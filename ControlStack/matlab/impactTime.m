
g = -9.81;

A = [zeros(3) eye(3); zeros(3) zeros(3)];
B = [zeros(5,1); g];

syms Ts

phi = expm([A B; zeros(1,7)]*Ts);


