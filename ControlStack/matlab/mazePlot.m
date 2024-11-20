% Read the CSV file containing the maze vertices
filename = '../build/maze_vertices.csv';

% Read the data from the CSV file
data = readmatrix(filename);

% Extract the columns: x1, y1, x2, y2
x1 = data(:, 1);
y1 = data(:, 2);
x2 = data(:, 3);
y2 = data(:, 4);
x3 = data(:, 5);
y3 = data(:, 6);
x4 = data(:, 7);
y4 = data(:, 8);

% Plot the maze
figure(1);
clf
hold on;

% Plot each wall segment
for i = 1:length(x1)
    rectangle('Position', [x1(i), y1(i), x3(i) - x2(i), y2(i) - y1(i)], 'FaceColor', 'k')
end

% Set axis equal to maintain the correct aspect ratio
axis equal;

% Set the title and labels
title('Maze Representation');
xlabel('X (meters)');
ylabel('Y (meters)');

% Display the grid for better visualization (optional)
grid on;

hold off;
