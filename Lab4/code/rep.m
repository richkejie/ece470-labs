function tau = rep(q, myrobot, obs)

tau = zeros(6,1);
F_rep = zeros(3,6);

% loop through links
% all joints are revolute (R)
for i =1:6
    % find Jacobian (same as att.m)
    J_v_i = zeros(3,6);

    % extract transformation
    H_i = myrobot.A(1:i, q);
    o_i = H_i.t; % origin of Fi wrt F0

    % fill columns of J until column i
    for j = 1:i
        H_j_1 = myrobot.A(1:j-1,q);
        R_j_1 = H_j_1.R;
        z_j_1 = R_j_1(:,3); % z_{j-1} wrt F0
        o_j_1 = H_j_1.t; % origin of F{j-1} wrt F0
        J_v_i(:,j) = cross(z_j_1, (o_i-o_j_1)); % column
    end

    % find F_i_rep
    F_i_rep = zeros(3,1);

    % cylinder with finite height
    if strcmp(obs.type, 'cyl')
        c = obs.c;
        c(3) = obs.h;
        n = [0;0;1];
        v_parr = (o_i - c) - (o_i - c)'*n*n;
        d_perp = max(0, (o_i - c)' *n);
        d_parr = max(0, norm(v_parr)-obs.R);
        dist = sqrt(d_perp^2 + d_parr^2);
        v_dist = d_perp * n + d_parr * (v_parr)/norm(v_parr);
        if dist == 0
            % outside of the region of influence
        elseif dist > obs.rho0
            F_i_rep = zeros(3,1);
            % in the region of influence
        else
            F_i_rep = 1 * (1/dist - 1/obs.rho0) * dist^(-3) * v_dist;
        end
    elseif strcmp(obs.type, 'sph')
        r = o_i - obs.c;
        distance = norm(r) - obs.R;
        if distance > obs.rho0 % outside radius of influence
            F_i_rep = zeros(3,1);
        else % inside radius of influence
            eta_i = 1;
            grad_distance = r / norm(r);
            F_i_rep = eta_i * (1/distance - 1/obs.rho0) * distance^(-2) * grad_distance;
        end
    end
         
    F_i_rep_display = F_i_rep*10^6; % display rep force
    tau = tau + transpose(J_v_i) * F_i_rep;
    F_rep(:,i) = F_i_rep;

end

% normalize tau
if norm(tau) ~= 0
    tau = tau/norm(tau);
end

tau = transpose(tau);

F_rep
tau

end