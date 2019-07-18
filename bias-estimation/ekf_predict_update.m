function ekf = ekf_predict_update(ekf, dt, z, args)
    x = ekf.x;
    P = ekf.P;
    
    F = ekf.F;

    H = ekf.H;

    x_prior = ekf.f(x, dt);
    P_prior = F(x, dt) * P * F(x, dt)' + ekf.Q;

    S = H(x_prior, args) * P_prior * H(x_prior, args)' + ekf.R;
    K = P_prior * H(x_prior, args)' / S;
    residual = z - ekf.h(x_prior, args);
    x = x_prior + K * residual;
    I_KH = eye(ekf.dim_x) - K * H(x_prior, args);
    P = I_KH * P_prior * I_KH' + K * ekf.R * K';

    ekf.x = x;
    ekf.P = P;

    ekf.result.x = [ekf.result.x x(1:4)];
    ekf.result.b = [ekf.result.b x(5:6)];