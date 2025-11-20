function tau = att(q,q2,myrobot)
    tau = zeros(6,1);
    F_att = zeros(3,6);

    % loop through links
    % all joints are revolute (R)
    for i =1:6
        % find Jacobian
        J_v_i = zeros(3,6);

        % extract transformation
        H_i = myrobot.A(1:i, q);
        o_i = H_i(1:3,4); % origin of Fi wrt F0

        % fill columns of J until column i
        for j = 1:i
            H_j_1 = myrobot.A(1:j-1,q);
            R_j_1 = H_j_1(1:3,1:3);
            z_j_1 = R_j_1(:,3); % z_{j-1} wrt F0
            o_j_1 = H_j_1(1:3,4); % origin of F{j-1} wrt F0
            J_v_i(:,j) = cross(z_j_1, (o_i-o_j_1)); % column
        end
        
        H_i_f = myrobot.A(1:i, q2);
        o_i_f = H_i_f(1:3,4); % final joint angle
        
        zeta_i = 1; % assume scaling constant is just 1 (can modify later)
        F_i_att = -zeta_i*(o_i - o_i_f);
        % F_i_att_display = F_i_att / 100
        tau = tau + transpose(J_v_i) * F_i_att;

        F_att(:,i) = F_i_att;
    end

    % normalize tau
    if norm(tau) ~= 0
        tau = tau/norm(tau);
    end

    tau = transpose(tau);
end