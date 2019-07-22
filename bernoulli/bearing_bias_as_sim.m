% Sequential Bernoulli filter with augmented states
%
% Two bearing-only sensors with biases
% estimate a CV target.
% Target birth and death considered
% Clutter considered
%
% 2D scenario

rng(0);

tic
% --- basic parameters
model.x_dim= 6;   %dimension of state vector
model.z_dim= 1;   %dimension of observation vector

% --- dynamical model parameters (CV model with 2 biases)
model.T= 1;                                     %sampling period
model.A0= [ 1 model.T; 0 1 ];                         %transition matrix                     
model.F= [ model.A0 zeros(2,4); zeros(2,2) model.A0 zeros(2,2); zeros(2,4) eye(2) ];
model.sigma_v = 5;
% std of sensor 1 bias
q_b1 = 1e-5;
% std of sensor 2 bias
q_b2 = 1e-5;
model.B0= [ (model.T^2)/2; model.T ];
model.B= [ model.B0 zeros(2,3); zeros(2,1) model.B0 zeros(2); zeros(2) diag([q_b1/model.sigma_v q_b2/model.sigma_v]) ];
model.Q= (model.sigma_v)^2* model.B*model.B';   %process noise covariance

% survival/death parameters
model.P_S= .99;
model.Q_S= 1-model.P_S;

% birth parameters (LMB birth model, single component only)
model.T_birth= 1;         %no. of LMB birth terms

model.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model.r_birth(1)=0.01;                                                          %prob of birth
model.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model.m_birth{1}(:,1)= [ 0.1; 0; 0.1; 0; 0; 0];                                 %mean of Gaussians
model.B_birth{1}(:,:,1)= diag([ 100; 10; 100; 10; 0.02; 0.02]);                         %std of Gaussians
model.P_birth{1}(:,:,1)= model.B_birth{1}(:,:,1)*model.B_birth{1}(:,:,1)';      %cov of Gaussians

% --- observation model parameters (noisy joint bearing) ---
% measurement transformation 
model.hx{1} = @bearing_bias1_hx;
model.hx{2} = @bearing_bias2_hx;
model.H{1} = @bearing_bias1_H;
model.H{2} = @bearing_bias2_H;
model.D= diag([ 2*(pi/180)]);      %std for angle and range noise
model.R= model.D*model.D';              %covariance for observation noise

% detection parameters
model.P_D= .98;   %probability of detection in measurements
model.Q_D= 1-model.P_D; %probability of missed detection in measurements

% clutter parameters
%poisson average rate of uniform clutter (per scan)
% If too many clutters exist, the target can't be estimated.
model.lambda_c= 0.2;                             
model.range_c= [ -pi/2 pi/2];          %uniform clutter on r/theta
model.pdf_c= 1/prod(model.range_c(:,2)-model.range_c(:,1)); %uniform clutter density

% Truth initialization
xstart = [0 3 0 9 1/57.3 -1/57.3]';
tbirth = 10;
tdeath = 80;

% Sensor setting
% no. of sensors
meas.S = 2;
meas.sensor_pos = [0 0; 200 -200]';

% Filter initialization
est.m = [0.1 0 0.1 0 0 0]';
est.P = diag([100 10 100 10 1/57.3 1/57.3]).^2;

% --- BERNOULLI FILTER START ---
truth= gen_truth(model, xstart, tbirth, tdeath);
meas=  gen_meas(model,truth,meas);
est=   run_filter(model,meas, est);
toc

handles= plot_results(model,truth,meas,est);
 
% --- HELPER FUNCTIONS ---
function Z = bearing_bias1_hx(X, sensor_pos)
% X: array(state, target)
    P= X([1 3],:) - sensor_pos;
    bias = X(5,:);

    Z(1,:)= atan2(P(1,:),P(2,:)) + bias;   
end

function [H,U]= bearing_bias1_H(X, sensor_pos)

    p= X([1 3],:) - sensor_pos;
    mag= p(1)^2 + p(2)^2;

    H= [ p(2)/mag  0  -p(1)/mag  0 1 0];
    U= eye(1);
end

function Z = bearing_bias2_hx(X, sensor_pos)
% X: array(state, target)
    P= X([1 3],:) - sensor_pos;
    bias = X(6,:);

    Z(1,:)= atan2(P(1,:),P(2,:)) + bias;   
end

function [H,U]= bearing_bias2_H(X, sensor_pos)

    p= X([1 3],:) - sensor_pos;
    mag= p(1)^2 + p(2)^2;

    H= [ p(2)/mag  0  -p(1)/mag  0 0 1];
    U= eye(1);
end
