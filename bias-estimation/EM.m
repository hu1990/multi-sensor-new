% Used for inplement EM method for linear system identification.
% This example is from `An explanation of the Expectation Maximization Algorithm`.
% The MATLAB code in the paper and the relavent toolbox are referenced.

rng(0);
max_iterations = 100;
min_ll_decrease = 1e-6; % Min decrease of log-likelihood
guess = 0.1; % Initial guess for the parameter

% --- REAL SYSTEM ---
A = 0.9;
N = 500;
x = zeros(1, N+1);
y = zeros(1, N);
C = 0.5;
Q = 0.1;
R = 0.1;

for t = 1:N
    x(t+1) = A * x(t)+ Q^0.5*randn();
    y(t) = C * x(t) + R^0.5*randn();
end

% --- EM ALGORITHM ---
LL = [];
a = [guess];

X1 = 0;
P1 = 0;

aNow = [0:0.01:0.79, 0.8:0.001:0.999, 1:0.01:1.3]';
len_aNow = size(aNow, 1);
Q_func = [];

% Filter model
kf.C = C;
kf.Q = Q;
kf.R = R;

for k = 1:max_iterations
    % E step
    % Kalman filter and smoother
    kf.A = guess;
    kf = Kalman(kf, y, X1, P1, N, true);
    LL = [LL -0.5*kf.LL];

    % Compute the Q function
    psi = 0;
    phi = 0;
    alpha = 0;
    beta = 0;
    for t = 1:N
        psi = psi + kf.x_smooth(:,t) * kf.x_smooth(:,t+1) + kf.M_smooth(1,1,t);
        phi = phi + kf.x_smooth(:,t) ^ 2 + kf.P_smooth(:,:,t);
    
        alpha = alpha + kf.x_smooth(t+1) * kf.x_smooth(t+1) + kf.P_smooth(1,1,t+1)^2;
        beta = beta + y(t) * kf.x_smooth(t);
    end

    % ???
    Q_k = zeros(len_aNow, 1);
    for i = 1:len_aNow
        Q_k(i) = -N * log(0.2*pi) - 5*sum(y.^2) - 5*alpha + 5*beta + 10*psi*aNow(i) - 5*phi*(aNow(i)^2+1/4);
    end
    Q_func = [Q_func Q_k];

    % M step
    guess = psi/phi;
    a = [a guess];

    % Check termination condition
    if k>1
        if LL(k) - LL(k-1) < min_ll_decrease
            break
    	end
    end
end

disp(a)
disp(k)
LL1 = zeros(len_aNow, 1);
myLL = zeros(len_aNow, 1);
% Compute the log-likelihood function
% for i = 1:len_aNow
%     A = aNow(i);
%     g_LL, g_Ri, g_xp = sqrt_kalman_smoother(y, A, C, Q, R, X1, P1, N, False);
%     LL1(i) = -0.5*g_LL;
%     % ???
%     Ltmp = -(N/2)*log(2*pi);
%     for t = 1:N
%         cov = g_Ri(t)^2;
%         Lpart1 = -(1/2)*((y(t) - C * g_xp(t))^2)/cov;
%         Lpart2 = -(1/2)*log(det(cov));
%         Ltmp = Ltmp + Lpart1 + Lpart2;
%     end
%     myLL(i) = Ltmp;
% end

% --- PLOT ---
% for x = [0,1,2,10]
%     figure()
%     plot(aNow, myLL, label='Log-likelihood')
%     plot(aNow, Q_func(x), label='Q function')
%     iL = np.argmax(myLL)
%     mL = myLL(iL)
%     iL = aNow(iL)
%     plt.plot((iL, iL), (-700, mL + 40))
%     iQ = np.argmax(Q_func(x))
%     mQ = Q_func(x)(iQ)
%     iQ = aNow(iQ)
%     plt.plot((iQ, iQ), (-700, mQ + 40))
%     plt.xlabel('Parameter a')
%     plt.ylabel('Log-likelihood and Q funciton')
%     plt.legend()
% end
% 
% plt.figure()
% plt.plot((ia(0)(0) for ia in a), label='Parameter estimates')
% plt.plot((0.9 for _ in a), label='True value')
% plt.xlabel('Iteration')
% plt.ylabel('Parameter a')
% plt.legend()
% 
% plt.figure()
% plt.plot(aNow, LL1)
% plt.plot(aNow, myLL)
% plt.legend()
% 
% plt.show()
