function model= gen_model

% basic parameters
model.x_dim= 4;   %dimension of state vector
model.z_dim= 2;   %dimension of observation vector

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

% observation model parameters (noisy r/theta only)
% measurement transformation given by gen_observation_fn, observation matrix is N/A in non-linear case
model.D= diag([ 2*(pi/180); 10 ]);      %std for angle and range noise
model.R= model.D*model.D';              %covariance for observation noise

% detection parameters
model.P_D= .98;   %probability of detection in measurements
model.Q_D= 1-model.P_D; %probability of missed detection in measurements

% clutter parameters
model.lambda_c= 20;                             %poisson average rate of uniform clutter (per scan)
model.range_c= [ -pi/2 pi/2; 0 2000 ];          %uniform clutter on r/theta
model.pdf_c= 1/prod(model.range_c(:,2)-model.range_c(:,1)); %uniform clutter density




