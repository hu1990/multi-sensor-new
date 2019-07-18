function model= gen_model

% basic parameters
model.x_dim= 4;   %dimension of state vector
model.z_dim= 1;   %dimension of observation vector

% dynamical model parameters (CV model)
model.T= 1;                                     %sampling period
model.A0= [ 1 model.T; 0 1 ];                         %transition matrix                     
model.F= [ model.A0 zeros(2,2); zeros(2,2) model.A0 ];
model.B0= [ (model.T^2)/2; model.T ];
model.B= [ model.B0 zeros(2,1); zeros(2,1) model.B0 ];
model.sigma_v = 5;
model.Q= (model.sigma_v)^2* model.B*model.B';   %process noise covariance

% survival/death parameters
model.P_S= .99;
model.Q_S= 1-model.P_S;

% birth parameters (LMB birth model, single component only)
model.T_birth= 1;         %no. of LMB birth terms

model.L_birth(1)=1;                                                             %no of Gaussians in birth term 1
model.r_birth(1)=0.01;                                                          %prob of birth
model.w_birth{1}(1,1)= 1;                                                       %weight of Gaussians - must be column_vector
model.m_birth{1}(:,1)= [ 0.1; 0; 0.1; 0];                                 %mean of Gaussians
model.B_birth{1}(:,:,1)= diag([ 100; 10; 100; 10]);                         %std of Gaussians
model.P_birth{1}(:,:,1)= model.B_birth{1}(:,:,1)*model.B_birth{1}(:,:,1)';      %cov of Gaussians

% --- observation model parameters (noisy joint bearing) ---
% measurement transformation 
model.hx = @bearing_hx;
model.H = @bearing_H;
model.D= diag([ 2*(pi/180)]);      %std for angle and range noise
model.R= model.D*model.D';              %covariance for observation noise

% detection parameters
model.P_D= .98;   %probability of detection in measurements
model.Q_D= 1-model.P_D; %probability of missed detection in measurements

% clutter parameters
model.lambda_c= 0.2;                             %poisson average rate of uniform clutter (per scan)
model.range_c= [ -pi/2 pi/2];          %uniform clutter on r/theta
model.pdf_c= 1/prod(model.range_c(:,2)-model.range_c(:,1)); %uniform clutter density


function Z = joint_bearing_hx(X, sensor_pos)
	sensor1_pos = sensor_pos(1:2);
	sensor2_pos = sensor_pos(3:4);

    P= X([1 3],:) - sensor1_pos;
    P2= X([1 3],:) - sensor2_pos;

    Z(1,:)= atan2(P(1,:),P(2,:));   
    Z(2,:)= atan2(P2(1,:),P2(2,:));   

function [H,U]= joint_bearing_H(X, sensor_pos)
	sensor1_pos = sensor_pos(1:2);
	sensor2_pos = sensor_pos(3:4);

    p= X([1 3],:) - sensor1_pos;
    p2= X([1 3],:) - sensor2_pos;
    mag= p(1)^2 + p(2)^2;
    mag2= p2(1)^2 + p2(2)^2;

    H= [ p(2)/mag  0  -p(1)/mag  0; ...
        p2(2)/mag2  0  -p2(1)/mag2  0];
    U= eye(2);
 
function Z = bearing_hx(X, sensor_pos)
% X: array(state, target)
    P= X([1 3],:) - sensor_pos;

    Z(1,:)= atan2(P(1,:),P(2,:));   

function [H,U]= bearing_H(X, sensor_pos)

    p= X([1 3],:) - sensor_pos;
    mag= p(1)^2 + p(2)^2;

    H= [ p(2)/mag  0  -p(1)/mag  0];
    U= eye(1);
