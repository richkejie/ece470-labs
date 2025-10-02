function [q] = inverse(H,myrobot)
    
    q = zeros(1,6);

    % get desired position of end effector
    o_d = H(1:3,4);

    % get R_d
    R_d = H(1:3, 1:3);

    % get needed DH parameters
    d = myrobot.d;
    a = myrobot.a;

    % get position of wrist
    o_c = o_d - R_d*[0; 0; d(6)];
    x_c = o_c(1);
    y_c = o_c(2);
    z_c = o_c(3);

    % intermediate calculations
    ALPHA = asin( abs(d(2)) / real(sqrt(x_c^2+y_c^2)) );
    R = real(sqrt( x_c^2 + y_c^2 - (d(2))^2 ));
    % S = abs( z_c - d(1) );
    S = z_c - d(1); % want to preserve direction
    D = ( R^2 + S^2 - (a(2))^2 - (d(4))^2 ) / ( 2 * a(2) * d(4) );

    % theta1:
    q(1,1) = atan2(y_c,x_c) - ALPHA;
    % theta3:
    q(1,3) = atan2(D, sqrt(1-D^2));

    % more intermediate calculations
    GAMMA = atan2( d(4)*sin( q(1,3)-pi/2 ) , a(2) + d(4)*cos( q(1,3)-pi/2 ) );

    % theta2:
    q(1,2) = atan2(S,R) - GAMMA;

    % get R_6_3
    joint = [q(1,1); q(1,2); q(1,3);];
    H_3_0 = forward(joint,myrobot);
    R_3_0 = H_3_0(1:3,1:3);
    R_6_3 = transpose(R_3_0)*R_d;

    % for some reason, expected theta4 in 5th pos and theta5 in 4th pos...
    % theta4:
    q(1,5) = atan2(real(sqrt(1-(R_6_3(3,3))^2)),R_6_3(3,3));
    % theta5:
    q(1,4) = atan2(R_6_3(2,3),R_6_3(1,3));
    % theta6:
    q(1,6) = atan2(R_6_3(3,2),-R_6_3(3,1));
end