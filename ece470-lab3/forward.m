function [H] = forward(joint,myrobot)
    % get DH parameters
    d = myrobot.d;
    alpha = myrobot.alpha;
    a = myrobot.a;
    
    s = size(joint);
    num_joints = s(1);

    % compute H_i-1_i using DH parameters
    A = zeros(num_joints,4,4);
    for i=1:num_joints
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
    H = eye(4);
    for i=1:num_joints
        H = H*squeeze(A(i,:,:));
    end
end