function sim_data = cv_bias_2d_sim(smart_sensor)
rng(0);
% Simulation setting
dt = 1;
total_time = 20;
iters = total_time/dt;

% Define targets
target.pos = [200000, 0]';
target.vel = [-4200, 5]';
target.pos_history = [target.pos];

% Define sensors
sensor1.pos = [0, 0]';
sensor1.vel = [3000, 0]';
sensor1.pos_history = [sensor1.pos];
sensor1.bias = 10/60/57.3;
sensor1.std = 1e-5;

% This relative position can give a better result.
% sensor2.pos = [500, 0]';
% While this cannot.
sensor2.pos = [0, 5000]';
sensor2.vel = [3000, 0]';
sensor2.pos_history = [sensor2.pos];
sensor2.bias = -15/60/57.3;
sensor2.std = 1e-5;

% Define filter
otsekf.x = [199000, 0, -6000, 0, 0, 0]';
otsekf.P = diag([1e3, 10, 1200, 10, 1e-2, 1e-2])^2;

otsekf.f = @fx;

otsekf.F = @F;

q = 1e-5;
Qx = [dt^3/3 0 dt^2/2 0;
	dt^2/2 0 dt 0;
	0 dt^3/3 0 dt^2/2;
	0 dt^2/2 0 dt]*q;
Qb = diag([1e-6, 1e-6])^2;
otsekf.Q = [Qx zeros(4,2);
    zeros(2,4) Qb];

otsekf.h = @two_bearing_sensor2d_hx;

otsekf.H = @two_bearing_sensor2d_H;

otsekf.R = diag([1e-5, 1e-5])^2;

otsekf.result.x = [otsekf.x(1:4)];
otsekf.result.b = [otsekf.x(5:6)];

% helper attributes
otsekf.dim_x = size(otsekf.x, 1);
otsekf.dim_z = size(otsekf.R, 1);

% Start simulation
for iter = 1:iters
    % Target motion
    target = cv_motion(target, dt);
    
    % Sensor motion
    sensor1 = cv_motion(sensor1, dt);
    if smart_sensor
        sensor2.vel = optimal_vel(target, sensor1, sensor2);
    end
    sensor2 = cv_motion(sensor2, dt);
    
    % Generate measurement
    obs = [bearing_measure(sensor1, target);
    	bearing_measure(sensor2, target)];

    % Filter
    args.sensor1_pos = sensor1.pos;
    args.sensor2_pos = sensor2.pos;
    otsekf = ekf_predict_update(otsekf, dt, obs, args);
end

% Generate data
time = 0:dt:dt*iters;

% Plot
% Target trajectory
% figure
% plot(time, target.pos_history(2,:));

% Return simulation result
sim_data.time = time;
sim_data.target_pos = target.pos_history;
sim_data.sensor1_pos = sensor1.pos_history;
sim_data.sensor2_pos = sensor2.pos_history;
sim_data.x = otsekf.result.x;
sim_data.b = otsekf.result.b;
sim_data.sensor1_bias = sensor1.bias;
sim_data.sensor2_bias = sensor2.bias;
sim_data.target_vel = target.vel;
end

function f = F(x, dt)
    f = eye(6);
    f(1,3) = dt;
    f(2,4) = dt;
end
function f = fx(x, dt)
    f = F(x, dt) * x;
end
function h = two_bearing_sensor2d_hx(x, args)
    sensor1_pos = args.sensor1_pos;
    sensor2_pos = args.sensor2_pos;
    target_pos = x(1:2);
    x1 = target_pos - sensor1_pos;
    x2 = target_pos - sensor2_pos;
    h = [atan2(x1(2), x1(1)) + x(5);
        atan2(x2(2), x2(1)) + x(6)];
end
function H = two_bearing_sensor2d_H(x, args)
    sensor1_pos = args.sensor1_pos;
    sensor2_pos = args.sensor2_pos;
    target_pos = x(1:2);
    x1 = target_pos - sensor1_pos;
    mag1 = norm(x1)^2;
    x2 = target_pos - sensor2_pos;
    mag2 = norm(x2)^2;
    H = [-x1(2)/mag1 x1(1)/mag1 0 0 1 0;
        -x2(2)/mag2 x2(1)/mag2 0 0 0 1];
end
function target = cv_motion(target, dt)
    target.pos = target.pos + dt * target.vel;
    target.pos_history = [target.pos_history target.pos];
end

function obs = bearing_measure(sensor, target)
    x = target.pos - sensor.pos;
    obs = atan2(x(2), x(1)) + sensor.bias + sensor.std *randn;
end
