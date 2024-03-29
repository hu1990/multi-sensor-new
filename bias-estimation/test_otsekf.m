% This is the reimplement of `test_OTSKF.py`
rng(0);
% Simulation setting
dt = 0.01;
total_time = 20;
iters = total_time/dt;

% Define targets
target.pos = [0, 200000]';
target.vel = [5, -7200]';
target.pos_history = [target.pos];

% Define sensors
sensor1.pos = [0, 0]';
sensor1.bias = 10/60/57.3;
sensor1.std = 1e-5;

% This relative position can give a better result.
sensor2.pos = [500, 0]';
% While this cannot.
% sensor2.pos = [5000, 0]';
sensor2.bias = -15/60/57.3;
sensor2.std = 1e-5;

% Define filter
otsekf.x = [0, 0, 199000, -6000]';
otsekf.P = diag([10, 10, 1e3, 1200])^2;

otsekf.b = [0, 0]';
otsekf.Pb = diag([1e-2, 1e-2])^2;


otsekf.f = @CV_2d_fx;



otsekf.F = @CV_2d_F;

q = 1e-5;
otsekf.Qx = [dt^3/3 dt^2/2 0 0;
	dt^2/2 dt 0 0;
	0 0 dt^3/3 dt^2/2;
	0 0 dt^2/2 dt]*q;
otsekf.Qb = diag([1e-6, 1e-6])^2;


otsekf.h = @two_bearing_sensor2d_hx;


otsekf.H = @two_bearing_sensor2d_H;

otsekf.R = diag([1e-5, 1e-5])^2;

otsekf.result.b =[otsekf.b];

% helper attributes
otsekf.dim_x = size(otsekf.x, 1);
otsekf.dim_z = size(otsekf.b, 1);

otsekf.V = zeros(otsekf.dim_x, otsekf.dim_z);

otsekf.xx = otsekf.x;
otsekf.Px = otsekf.P;

otsekf.B = zeros(otsekf.dim_x, otsekf.dim_z);
otsekf.C = eye(otsekf.dim_z);
otsekf.D = eye(otsekf.dim_z);

% Start simulation
for iter = 1:iters
    % Target motion
    target = target_motion(target, dt);

    % Generate measurement
    obs = [bearing_measure(sensor1, target);
    	bearing_measure(sensor2, target)];

    % Filter
    otsekf = otsekf_predict_update(otsekf, dt, obs, sensor2.pos);
end

% Generate data
time = 0:dt:dt*iters;

figure
plot(time, target.pos_history(2,:));
% Plot
figure;
hold on
plot([time(1), time(end)], [sensor1.bias, sensor1.bias]);
plot(time, otsekf.result.b(1,:));
legend('bias1', 'estimate1');

figure;
hold on
plot([time(1), time(end)], [sensor2.bias, sensor2.bias]);
plot(time, otsekf.result.b(2,:));
function h = two_bearing_sensor2d_hx(x, sensor2_pos)
    x1 = x(1:2:3);
    x2 = x1 - sensor2_pos;
    h = [atan2(x1(1), x1(2));
        atan2(x2(1), x2(2))];
end
function H = two_bearing_sensor2d_H(x, sensor2_pos)
    x1 = x(1:2:3);
    mag = norm(x1)^2;
    x2 = x1 - sensor2_pos;
    mag2 = norm(x2)^2;
    H = [x1(2)/mag 0 -x1(1)/mag 0;
        x2(2)/mag2 0 -x2(1)/mag2 0];
end
function target = target_motion(target, dt)
    target.pos = target.pos + dt * target.vel;
    target.pos_history = [target.pos_history target.pos];
end

function obs = bearing_measure(sensor, target)
    x = target.pos - sensor.pos;
    obs = atan2(x(1), x(2)) + sensor.bias + sensor.std *randn;
end
