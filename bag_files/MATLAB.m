% Specifica il file direttamente
filename = 'rosbag2_circular_trapezoidal.db3';
bagReader = ros2bagreader(filename);
msgs = readMessages(bagReader);

% Estrai gli efforts in modo corretto
efforts = zeros(length(msgs), 7);  
for i = 1:length(msgs)
   efforts(i,:) = double(msgs{i}.data);  
end

% Parametri temporali
t_start = 0;
num_points = size(efforts, 1);
t_end = 4.0;  
time_vector = linspace(t_start, t_end, num_points);

% Plot
figure('Position', [100 100 1200 600]);
colors = [
   0.1216    0.4667    0.7059  % Blu acceso
   0.8392    0.1529    0.1569  % Rosso vivo
   0.1725    0.6275    0.1725  % Verde brillante
   0.5804    0.4039    0.7412  % Viola
   1.0000    0.4980    0.0549  % Arancione
   0.1490    0.1490    0.1490  % Nero
   0.9290    0.6940    0.1250  % Giallo oro
];

% Plot each joint effort
hold on;
legendEntries = cell(1, 7);
for i = 1:7
   plot(time_vector, efforts(:,i), 'Color', colors(i,:), 'LineWidth', 2, 'Marker', '.');
   legendEntries{i} = sprintf('Joint %d', i);
end

% Customize plot
xlabel('Time (s)', 'FontSize', 12);
ylabel('Torque (Nm)', 'FontSize', 12);
title('Joint Torques vs Time - Circular Trajectory with Trapezoidal Velocity', 'FontSize', 14);
legend(legendEntries, 'Location', 'eastoutside');
grid on;

% Add information box
dim = [.15 .6 .3 .3];
str = sprintf('File: %s\nDuration: %.2f s\nSamples: %d', ...
   filename, t_end, num_points);
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', ...
   'BackgroundColor', 'white', 'EdgeColor', 'none');

hold off;