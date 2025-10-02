function [H] = forward(joint,myrobot)
    % get DH parameters
    d = myrobot.d;
    alpha = myrobot.alpha;
    a = myrobot.a;

    % compute H_i-1_i using DH parameters
    A = zeros(6,4,4);
    for i=1:6
        theta_i = joint(i);
        alpha_i = alpha(i);
        a_i = a(i);
        d_i = d(i);
        R =     [cos(theta_i)      -sin(theta_i)*cos(alpha_i)      sin(theta_i)*sin(alpha_i);
                 sin(theta_i)       cos(theta_i)*cos(alpha_i)     -cos(theta_i)*sin(alpha_i);
                           0                     sin(alpha_i)                   cos(alpha_i)];
        D =     [a_i*cos(theta_i);
                 a_i*sin(theta_i);
                             d_i];
        A(i,:,:) = [  R   D;
                    0 0 0 1];
    end
    
    % get H_0_6
    H = squeeze(A(1,:,:))*squeeze(A(2,:,:))*squeeze(A(3,:,:))*squeeze(A(4,:,:))*squeeze(A(5,:,:))*squeeze(A(6,:,:));
end