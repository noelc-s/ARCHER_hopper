
% Perm 1 -- sim
unit_1 = [0.0776 0 0.0547];
unit_2 = [-0.0388 0.0672 0.0548];
unit_3 = [-0.0388 -0.0672 0.0548];

% Perm 2
% unit_1 = [0.0776 0 0.0547];
% unit_3 = [-0.0388 0.0672 0.0548];
% unit_2 = [-0.0388 -0.0672 0.0548];

% Perm 3
% unit_2 = [0.0776 0 0.0547];
% unit_1 = [-0.0388 0.0672 0.0548];
% unit_3 = [-0.0388 -0.0672 0.0548];

% Perm 4
% unit_2 = [0.0776 0 0.0547];
% unit_3 = [-0.0388 0.0672 0.0548];
% unit_1 = [-0.0388 -0.0672 0.0548];

% Perm 5
% unit_3 = [0.0776 0 0.0547];
% unit_1 = [-0.0388 0.0672 0.0548];
% unit_2 = [-0.0388 -0.0672 0.0548];

% Perm 6
% unit_3 = [0.0776 0 0.0547];
% unit_2 = [-0.0388 0.0672 0.0548];
% unit_1 = [-0.0388 -0.0672 0.0548];

R1 = eul2rotm(unit_1,'XYZ');
R2 = eul2rotm(unit_2,'XYZ');
R3 = eul2rotm(unit_3,'XYZ');

X = unit_1/norm(unit_1);
Y = unit_2/norm(unit_2);
Z = unit_3/norm(unit_3);

R = [X' Y' Z'];

% eul_1 = eul_1/norm(eul_1);
% eul_2 = eul_2/norm(eul_2);
% eul_3 = eul_3/norm(eul_3);

% eul_center = unit_1 + [0 0 45*pi/180];

% fly_1 = eul2rotm(unit_1,'XYZ');
% fly_2 = eul2rotm(unit_2,'XYZ');
% fly_3 = eul2rotm(unit_3,'XYZ');

% center = eul2rotm(eul_center,'XYZ');

% x = [1 0 0];
% y = [0 1 0];
% z = [0 0 1];

% R = [unit_1; unit_2; unit_3]';

clf; axis equal;
hold on;
plot3([0 X(1)],[0 X(2)],[0 X(3)])
plot3([0 Y(1)],[0 Y(2)],[0 Y(3)])
plot3([0 Z(1)],[0 Z(2)],[0 Z(3)])

p1 = R*[1 0 0]';
p2 = R*[0 1 0]';
p3 = R*[0 0 1]';
% scatter3(p1(1),p1(2), p1(3))
% scatter3(p2(1),p2(2), p2(3))
% scatter3(p3(1),p3(2), p3(3))


tr = trace(R);
R(1,1) = R(1,1);
R(2,2) = R(2,2);
R(3,3) = R(3,3);

% https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
if (tr > 0)
  S = sqrt(tr+1.0) * 2;% S=4*qw 
  qw = 0.25 * S;
  qx = (R(3,2) - R(2,3)) / S;
  qy = (R(1,3) - R(3,1)) / S; 
  qz = (R(2,1) - R(1,2)) / S; 
elseif ((R(1,1) > R(2,2))&(R(1,1) > R(3,3))) 
  S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; % S=4*qx 
  qw = (R(3,2) - R(2,3)) / S;
  qx = 0.25 * S;
  qy = (R(1,2) + R(2,1)) / S; 
  qz = (R(1,3) + R(3,1)) / S; 
elseif (R(2,2) > R(3,3)) 
  S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; % S=4*qy
  qw = (R(1,3) - R(3,1)) / S;
  qx = (R(1,2) + R(2,1)) / S; 
  qy = 0.25 * S;
  qz = (R(2,3) + R(3,2)) / S; 
else 
  S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; % S=4*qz
  qw = (R(2,1) - R(1,2)) / S;
  qx = (R(1,3) + R(3,1)) / S;
  qy = (R(2,3) + R(3,2)) / S;
  qz = 0.25 * S;
end

q = [qw qx qy qz];
R_check = quat2rotm(q);
p1 = R_check*[1 0 0]';
p2 = R_check*[0 1 0]';
p3 = R_check*[0 0 1]';
scatter3(p1(1),p1(2), p1(3))
scatter3(p2(1),p2(2), p2(3))
scatter3(p3(1),p3(2), p3(3))

eul_check = quat2eul(q,'XYZ')
