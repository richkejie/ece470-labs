function [q] = inverse_kuka(H,myrobot)
    q = zeros(1,6);

    % get desired position of end effector
    o_d = H(1:3,4);

    % get R_d
    R_d = H(1:3, 1:3);

    % get needed DH parameters
    d = myrobot.d;
    a = myrobot.a;

    % get position of wrist
    o_c = o_d - R_d*[a(6); 0; d(6)];
    x_c = o_c(1);
    y_c = o_c(2);
    z_c = o_c(3);

    % theta1:
    q(1,1) = atan2(y_c,x_c);

    % intermediate calculations
    alpha = atan2(a(3),d(4));
    l = sqrt((d(4))^2 + a(3)^2);
    s = z_c - d(1);
    r = sqrt(x_c^2 + y_c^2) - a(1);
    D = (s^2 + r^2 - l^2 - a(2)^2) / (2*l*a(2));
    
    % theta3:
    q(1,3) = asin(D) - alpha;

    % more intermediate calculations
    psi2 = atan2(s, r);
    B = real(-l*cos(q(1,3) + alpha));
    C = real(l*sin(q(1,3) + alpha));
    psi1 = atan2(B, a(2)+C);

    % theta2:
    q(1,2) = psi2 - psi1;

    % get R_6_3
    joint = [q(1,1); q(1,2); q(1,3); 0; 0; 0];
    H_3_0 = forward_kuka(joint,myrobot);
    R_3_0 = H_3_0(1:3,1:3);
    R_6_3 = transpose(R_3_0)*R_d;

    % for some reason, expected theta4 in 5th pos and theta5 in 4th pos...
    % theta4:
    q(1,5) = atan2(real(sqrt(1-(R_6_3(3,3))^2)),real(R_6_3(3,3)));
    % theta5:
    q(1,4) = atan2(real(R_6_3(2,3)),real(R_6_3(1,3)));
    % theta6:
    q(1,6) = atan2(real(R_6_3(3,2)),real(-R_6_3(3,1)));
end