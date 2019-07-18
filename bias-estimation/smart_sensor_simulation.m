% Simulation scenario is as follows.

% Two bearing-only sensor cooperate to track a constant velocity target.
% The sensors are moving in a constant speed, too, while:
% - in Simulation 1, Sensor 2 moves as same as Sensor 1;
% - in Simulation 2, Sensor 2 decides an optimal heading angle.

% Filter method used is EKF actually.

% Simulation results are plotted.

% Simulations
sim_data1 = cv_bias_2d_sim(false);
sim_data2 = cv_bias_2d_sim(true);

% Save results
save('sim_data1', 'sim_data1');
save('sim_data2', 'sim_data2');

% Plot
smart_sensor_plot();