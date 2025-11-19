function [H] = forward_kuka(joint,myrobot)
    % get DH parameters
    d = myrobot.d;
    alpha = myrobot.alpha;
    a = myrobot.a;

    % compute H_i-1_i using DH parameters
    A = zeros(6,4,4);
    for i=1:6
        theta = joint(i);
        A(i,:,:) = [cos(theta) -sin(theta)*cos(alpha(i)) sin(theta)*sin(alpha(i)) a(i)*cos(theta);
                sin(theta) cos(theta)*cos(alpha(i)) -cos(theta)*sin(alpha(i)) a(i)*sin(theta);
                0          sin(alpha(i))         cos(alpha(i))             d(i);
                0 0 0 1];
    end
    
    % get H_0_6
    H = squeeze(A(1,:,:)) * squeeze(A(2,:,:)) * squeeze(A(3,:,:)) * squeeze(A(4,:,:)) * squeeze(A(5,:,:)) * squeeze(A(6,:,:));
end