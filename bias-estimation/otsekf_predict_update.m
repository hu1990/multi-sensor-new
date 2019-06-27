function otsekf = otsekf_predict_update(otsekf, dt, z, args)
    % dt: filter period
    % z: measurement
    % args: argments for system and measurement equations 
    Qx = otsekf.Qx;
    Qb = otsekf.Qb;
    R = otsekf.R;  
                  
    F = otsekf.F;
    Fx = otsekf.f;
    H = otsekf.H;
    Hx = otsekf.h;
    B = otsekf.B;
    C = otsekf.C;
    D = otsekf.D;

    x = otsekf.x;
    P = otsekf.P;
    x1 = otsekf.xx;
    P1 = otsekf.Px;
    b = otsekf.b;
    P2 = otsekf.Pb;
    V = otsekf.V;
    % --------------- PREDICT ----------------
    % bias filter
    b_prior = C * b;
    P2_prior = C * P2 * C' + Qb;
    % Get the transformation
    Ubar = F(x, dt) * V + B;
    U = Ubar * P2 * C' / P2_prior;
    % bias-free filter
    x1_prior = Fx(x, dt) + B * b - U * b_prior;
    P1_prior = F(x, dt) * P1 * F(x, dt)' + Qx + Ubar * P2 * Ubar' - U * P2_prior * U';
    % ---------------- UPDATE -----------------
    x_prior = x1_prior + U * b_prior;
    % bias filter
    S = H(x_prior, args) * U + D;
    K2 = P2_prior * S' / (S * P2_prior * S' + R + H(x_prior, args) * P1_prior * H(x_prior, args)');
    resi2 = (z - Hx(x_prior, args) - D * b_prior);
    b = b_prior + K2 * resi2;
    P2 = (eye(otsekf.dim_z) - K2 * S) * P2_prior;
    P2 = (P2 + P2')/2;
    % bias-free filter
    K1 = P1_prior * H(x_prior, args)' / (H(x_prior, args) * P1_prior * H(x_prior, args)' + R);
    x1 = x1_prior + K1 * (resi2 + S * b_prior);
    P1 = (eye(otsekf.dim_x) - K1 * H(x_prior, args)) * P1_prior;
    P1 = (P1 + P1')/2;
    % coupling
    V = U - K1 * S;
    x = x1 + V * b;
    P = P1 + V * P2 * V';
    P = (P + P')/2;
    % -------------------------------------------
    otsekf.x = x;
    otsekf.P = P;
    otsekf.xx = x1;
    otsekf.Px = P1;
    otsekf.b = b;
    otsekf.Pb = P2;
    otsekf.V = V;
    % Save the result
    otsekf.result.b = [otsekf.result.b b];
end
   

