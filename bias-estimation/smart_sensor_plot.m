% Load results
load('sim_data1', 'sim_data1');
load('sim_data2', 'sim_data2');

% Plot
path = 'Figures/';
% Trajectories of target and sensors in both cases.
f = figure;
f.Position = [100 100 270 200];
box on;
hold on;
plot(sim_data1.target_pos(1,:)/1000, sim_data1.target_pos(2,:)/1000, 'kx-', 'MarkerSize', 2);
plot(sim_data1.sensor1_pos(1,:)/1000, sim_data1.sensor1_pos(2,:)/1000, 'k.-');
plot(sim_data1.sensor2_pos(1,:)/1000, sim_data1.sensor2_pos(2,:)/1000, 'k:');
plot(sim_data2.sensor2_pos(1,:)/1000, sim_data2.sensor2_pos(2,:)/1000, 'k-');
legend('目标', '传感器1', '传感器2情形1', '传感器2情形2', 'Location', 'Southeast');
xlabel('x/km');
ylabel('y/km');
ylim([-100 50]);
xticks([0 50 100 150 200]);
% yticks([-50 0 50]);
print([path, 'trajectory.tif'], '-dtiff', '-r600');
% Bias estimation in both cases.
% figure;
% subplot(211);
% hold on;
% plot([sim_data1.time(1), sim_data1.time(end)], [sim_data1.sensor1_bias, sim_data1.sensor1_bias]);
% plot(sim_data1.time, sim_data1.b(1,:), sim_data2.time, sim_data2.b(1,:));
% legend('True', 'Estimation in Case 1', 'Estimation in Case 2');
% subplot(212);
% hold on;
% plot([sim_data1.time(1), sim_data1.time(end)], [sim_data1.sensor2_bias, sim_data1.sensor2_bias]);
% plot(sim_data1.time, sim_data1.b(2,:), sim_data2.time, sim_data2.b(2,:));
% legend('True', 'Estimation in Case 1', 'Estimation in Case 2');

% Bias estimation error in both cases.
% Bias estimate error of Sensor 1
f = figure;
f.Position = [100 100 260 120];
a = gca;
a.Position = [0.2,0.26,0.72,0.67];
box on;
hold on;
plot(sim_data1.time, (sim_data1.sensor1_bias - sim_data1.b(1,:))*57.3, 'k:');
plot(sim_data2.time, (sim_data2.sensor1_bias - sim_data2.b(1,:))*57.3, 'k-');
% plot([sim_data1.time(1), sim_data1.time(end)], [0 0], 'k--');
legend('情形1', '情形2');
xlabel('时间/s');
ylabel('b_1估计误差/\circ', 'Position', [-2.8 0.05]);
ylim([-0.05 0.15]);
yticks(-0.05:0.05:0.15);
print([path, 'b1_error.tif'], '-dtiff', '-r600');

% Bias estimate error of Sensor 2
f = figure;
f.Position = [100 100 260 120];
a = gca;
a.Position = [0.2,0.26,0.72,0.67];
box on;
hold on;
plot(sim_data1.time, (sim_data1.sensor2_bias - sim_data1.b(2,:))*57.3,'k:');
plot(sim_data2.time, (sim_data2.sensor2_bias - sim_data2.b(2,:))*57.3,'k-');
% plot([sim_data1.time(1), sim_data1.time(end)], [0 0], 'k--');
legend('情形1', '情形2', 'Location', 'SouthEast');
xlabel('时间/s');
ylabel('b_2估计误差/\circ', 'Position', [-2.8 -0.05]);
ylim([-0.2 0.1]);
print([path, 'b2_error.tif'], '-dtiff', '-r600');

% State estimation in both cases. (Target position)
% figure;
% subplot(211);
% hold on;
% plot(sim_data1.time, sim_data1.target_pos(1,:));
% plot(sim_data1.time, sim_data1.x(1,:), sim_data2.time, sim_data2.x(1,:));
% legend('True', 'Estimation in Case 1', 'Estimation in Case 2');
% subplot(212);
% hold on;
% plot(sim_data1.time, sim_data1.target_pos(2,:));
% plot(sim_data1.time, sim_data1.x(2,:), sim_data2.time, sim_data2.x(2,:));
% legend('True', 'Estimation in Case 1', 'Estimation in Case 2');

% State estimation error in both cases. (Target position)
% x position estimate error
f = figure;
f.Position = [100 100 260 120];
a = gca;
a.Position = [0.14,0.26,0.78,0.67];
box on;
hold on;
plot(sim_data1.time, (sim_data1.target_pos(1,:) - sim_data1.x(1,:))/1000, 'k:');
plot(sim_data2.time, (sim_data2.target_pos(1,:) - sim_data2.x(1,:))/1000, 'k-');
% plot([sim_data1.time(1), sim_data1.time(end)], [0 0], 'k--');
legend('情形1', '情形2');
xlabel('时间/s');
ylabel('x估计误差/km', 'Position', [-1.3268 1.2]);
ylim([-0.3 3]);
print([path, 'x_error.tif'], '-dtiff', '-r600');

% y estimate error
f = figure;
f.Position = [100 100 260 120];
box on;
hold on;
a = gca;
a.Position = [0.14,0.26,0.78,0.67];
plot(sim_data1.time, sim_data1.target_pos(2,:) - sim_data1.x(2,:), 'k:');
plot(sim_data2.time, sim_data2.target_pos(2,:) - sim_data2.x(2,:), 'k-');
% plot([sim_data1.time(1), sim_data1.time(end)], [0 0], 'k--');
legend('情形1', '情形2', 'Location', 'SouthEast');
xlabel('时间/s');
ylabel('y估计误差/m', 'Position', [-1.3268 0]);
print([path, 'y_error.tif'], '-dtiff', '-r600');

% State estimation in both cases. (Target velocity)
% figure;
% subplot(211);
% hold on;
% plot([sim_data1.time(1), sim_data1.time(end)], [sim_data1.target_vel(1), sim_data1.target_vel(1)]);
% plot(sim_data1.time, sim_data1.x(3,:), sim_data2.time, sim_data2.x(3,:));
% legend('True', 'Estimation in Case 1', 'Estimation in Case 2');
% subplot(212);
% hold on;
% plot([sim_data1.time(1), sim_data1.time(end)], [sim_data1.target_vel(2), sim_data1.target_vel(2)]);
% plot(sim_data1.time, sim_data1.x(4,:), sim_data2.time, sim_data2.x(4,:));
% legend('True', 'Estimation in Case 1', 'Estimation in Case 2');

% State estimation error in both cases. (Target velocity)
% v_x estimate error
f = figure;
f.Position = [100 100 260 120];
box on;
hold on;
a = gca;
a.Position = [0.14,0.26,0.78,0.67];
plot(sim_data1.time, (sim_data1.target_vel(1) - sim_data1.x(3,:))/1000, 'k:');
plot(sim_data2.time, (sim_data2.target_vel(1) - sim_data2.x(3,:))/1000, 'k-');
% plot([sim_data1.time(1), sim_data1.time(end)], [0 0], 'k--');
legend('情形1', '情形2');
xlabel('时间/s');
ylabel('v_x估计误差/(km/s)', 'Position', [-1.3268 1]);
print([path, 'vx_error.tif'], '-dtiff', '-r600');

% v_y estimate error
f = figure;
f.Position = [100 100 260 120];
box on;
hold on;
a = gca;
a.Position = [0.14,0.26,0.78,0.67];
plot(sim_data1.time, sim_data1.target_vel(2) - sim_data1.x(4,:), 'k:');
plot(sim_data2.time, sim_data2.target_vel(2) - sim_data2.x(4,:), 'k-');
% plot([sim_data1.time(1), sim_data1.time(end)], [0 0], 'k--');
legend('情形1', '情形2');
xlabel('时间/s');
ylabel('v_y估计误差/(m/s)', 'Position', [-1.3268 2]);
ylim([-2 6]);
print([path, 'vy_error.tif'], '-dtiff', '-r600');