function sensor2_vel = optimal_vel(target, sensor1, sensor2)
    % Get relative position and relative velocity
    sensor1_rel_pos = target.pos - sensor1.pos;
    sensor1_rel_vel = sensor1.vel - target.vel;
    sensor2_rel_pos = target.pos - sensor2.pos;
    sensor2_rel_vel = sensor2.vel - target.vel;

    r1 = norm(sensor1_rel_pos);
    r2 = norm(sensor2_rel_pos);
    d1 = norm(sensor1_rel_vel);
    d2 = norm(sensor2_rel_vel);
    q1 = atan(sensor1_rel_pos(2)/sensor1_rel_pos(1));
    q2 = atan(sensor2_rel_pos(2)/sensor2_rel_pos(1));
    phi1 = atan(sensor1_rel_vel(2)/sensor1_rel_vel(1));

    k1 = r1/d1;
    k2 = r2/d2;

    den_temp = cos(2*q1 - q2 - phi1) - k1 * cos(q1 - q2);

    num1 = (1 - k2^2) * (sin(2*q1 - q2 - phi1) - k1 * sin(q1 - q2));
    den1 = (1 + k2^2) * den_temp;

    num2 = (1 - k2^2) * sqrt(1 + k1^2 - 2 * k1 * cos(q1 - phi1));
    den2 = 2 * k2 * den_temp;

    % phi2 = q2 - atan(num1/den1) + atan(num2/den2);
    % The other solution is
    phi2 = q2 - atan(num1/den1) - atan(num2/den2);

    % Get absolute velocity
    sensor2_vel = [cos(phi2); sin(phi2)] * norm(sensor2.vel);