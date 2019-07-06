function kf = kalman_smoother(kf, z, X1, P1, N, smooth)
    % kf: Kalman filter`
    % z: measurement
    % X1: initial estimation
    % P1: initial covariance
    % N: number of steps
    % smooth: whether smooth or not

    % Get filter model
    A = kf.A;
    C = kf.C;
    Q = kf.Q;
    R = kf.R;

    % Get the dimensions
    n = size(X1, 1);
    m = size(z, 1);

    x_prior = zeros(n, N+1);
    x_prior(1) = X1;
    x = zeros(n, N);

    P_prior = zeros(n, n, N+1);
    P = zeros(n,n,N);

    S = zeros(m,m,N);
    K = zeros(n,m,N);
    z_prior = zeros(m,N);
    z_postier = zeros(m,N);
    residual = zeros(m,N);
    error = zeros(m,N);

    LL = 0;

    % Kalman filter
    for t = 1:N
        % --- UPDATE PHASE ---

        % Covariance update
        S(:,:,t) = C * P_prior(:,:,t) * C' + R;
        K(:,:,t) = P_prior(:,:,t) * C' / S(:,:,t);
        IKH = eye(n) - K(:,:,t) * C;
        P(:,:,t) = IKH * P_prior(:,:,t) * IKH' + K(:,:,t) * R * K(:,:,t)';

        % Means update
        z_prior(:,t) = C * x_prior(:,t);
        residual(:,t) = z(:,t) - z_prior(:,t);
        x(:,t) = x_prior(:,t) + K(:,t) * residual(:,t);

        % Likelihood (??)
        Riep = S(:,:,t) \ residual(t);
        LL = LL + residual(t)' / S(:,:,t) * residual(t) + sum(log(diag(S(:,:,t))));

        % error
        z_postier(:,t) = C * x(:,t);
        error(:,t) = z(:,t) - z_postier(:,t);

        % --- PREDICT PHASE ---

        % Means predict
        x_prior(:,t+1) = A * x(:,t);

        % Covariance predict
        P_prior(:,:,t+1) = A * P(:,:,t) * A' + Q;
    end

    if smooth
        % Smooth
        % Initialization
        x_smooth = zeros(n, N+1);
        x_smooth(:, N+1) = x_prior(:, N+1);
        x_smooth(:, N) = x(:, N);

        P_smooth = zeros(n,n,N+1);
        P_smooth(:,:,N+1) = P_prior(:,:,N+1);
        P_smooth(:,:,N) = P(:,:,N);

        M_smooth = zeros(n,n,N);
        M_smooth(:,:,N) = A * P(:,:,N);

        z_smooth = zeros(m,N);
        error_smooth = zeros(m,N);

        for t = N-1:-1:1
            % Calculate Jt
            AP = A * P(:,:,t);
            Jt = AP' / (P_prior(:,:,t+1));

            P_smooth(:,:,t) = P(:,:,t) + Jt * (P_smooth(:,:,t+1) - P_prior(:,:,t+1)) * Jt';

            % State smooth 
            x_smooth(:,t) = x(:,t) + Jt * (x_smooth(:,t+1) - x_prior(:,t+1));
            z_smooth(:,t) = C * x_smooth(:,t);
            error_smooth(:,t) = z(:,t) - z_smooth(:,t);

            if t == N-1
                M_smooth(:,:,t) = (eye(n) - K(:,:,N) * C) * AP;
            else
                % M_t|N = (P_t+1|t+1 + J_t+1 * (M_t+1|N - A * P_t+1|t+1)) * J_t'
                M_smooth(:,:,t) = (Pp1 + Jtp1 * (M_smooth(:,:,t+1) - APp1)) * Jt';
            end
            Jtp1 = Jt;
            APp1 = AP;
            Pp1 = P(:,:,t);
        end
        
        kf.x_prior = x_prior;
        kf.S = S;
        kf.x_smooth = x_smooth;
        kf.P_smooth = P_smooth;
        kf.M_smooth = M_smooth;
        kf.LL = LL;
    end
end