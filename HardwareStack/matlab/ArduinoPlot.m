% plot Arduino low level from log.txt -- Koios only
clear all; clc; 

% import recorded data from USB / micro sd card
USER = getenv("USERNAME");
A = readmatrix("/media/"+USER+"/KOIOS/log.txt");
[data_pts, data_size] = size(A);

% sizes of each data type
size_time = 1;
size_quat_a = 4;
size_quat_d = 4;
size_omeg_a = 3;
size_omeg_d = 3;
size_foot = 3;
size_currents = 3;

size_all = [size_time; 
            size_quat_a; 
            size_quat_d; 
            size_omeg_a; 
            size_omeg_d;
            size_foot;
            size_currents];

size_tot = sum(size_all);

titles = ["Time";
          "Quaternion Actual"; 
          "Quaternion Desired"; 
          "Angular Vel. Actual"; 
          "Angular Vel. Desired"; 
          "Foot State"; 
          "Motor Currents"];
% ylabels = ["Time";
%           ["$(q_a)_w$", "$(q_a)_x$", "$(q_a)_y$", "$(q_a)_z$"]; 
%           ["$(q_d)_w$", "$(q_d)_x$", "$(q_d)_y$", "$(q_d)_z$"]; 
%           ["$(\omega_a)_x$", "$(\omega_a)_y$", "$(\omega_a)_z$"];
%           ["$(\omega_d)_x$", "$(\omega_d)_y$", "$(\omega_d)_z$"];
%           ["Foot State"]; 
%           ["Motor Currents"]];

% ensure that CSV data is the same as expected data
assert(size_tot == data_size, "The recorded data is not " + ...
    "the expected size of the data");

% set indeces of each data type
idx_list = [];
idx=1;
for i=1:length(size_all)
    tmp = [idx, size_all(i)+idx-1];
    idx_list = [idx_list ; tmp];
    idx = idx + size_all(i);
end

% plot all data
data_time = (A(:,1) - (A(1,1))) / (10E6);
for j = 2:length(idx_list)
    idx_begin = idx_list(j,1);
    idx_end   = idx_list(j,2);
    
    figure(j); hold on;
    for k = idx_begin:idx_end
        plot(data_time,A(:,k), '.');
    end
    title(titles(j),Interpreter='latex')
    xlabel("Time",Interpreter='latex')
    grid on;
    hold off;
end






