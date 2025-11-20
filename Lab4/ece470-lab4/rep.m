function tau = rep(q, myrobot, obs, eta)

tau = zeros(6,1);
F_rep = zeros(3,6);

% loop through links
% all joints are revolute (R)
for i =1:6
    % find Jacobian (same as att.m)
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

    % find F_i_rep
    F_i_rep = zeros(3,1);

    eta_i = eta;

    % cylinder with finite height
    if strcmp(obs.type, 'cyl')
        c = obs.c;
        c(3) = obs.h;
        n = [0;0;1];
        d_perp = max(0, (o_i - c)'*n);         % perpendicular distance
        v_perp = d_perp * n;                    % perpendicular vector
        v_parr = (o_i - c) - (o_i - c)'*n*n;    % parallel vector
        d_parr = max(0, norm(v_parr)-obs.R);    % parallel distance
        v_parr = d_parr * (v_parr)/ norm(v_parr);
        
        distance = sqrt(d_perp^2 + d_parr^2);
        v_dist = v_perp + v_parr;
        if distance == 0
            % in collision
        elseif distance > obs.rho0 % outside radius of influence
            F_i_rep = zeros(3,1);
        else
            F_i_rep = eta_i * (1/distance - 1/obs.rho0) * distance^(-3) * v_dist;
        end
    elseif strcmp(obs.type, 'plane')
        distance = max(0, (o_i - obs.p)'*obs.n);
        if distance == 0
            % in collision
        elseif distance > obs.rho0 % outside influence
            F_i_rep = zeros(3,1);
        else
            F_i_rep = eta_i * (1/distance - 1/obs.rho0) * distance^(-2) * obs.n;
        end
    elseif strcmp(obs.type, 'sph')
        r = o_i - obs.c;
        distance = norm(r) - obs.R;
        if distance > obs.rho0 % outside radius of influence
            F_i_rep = zeros(3,1);
        else % inside radius of influence
            grad_distance = r / norm(r);
            F_i_rep = eta_i * (1/distance - 1/obs.rho0) * distance^(-2) * grad_distance;
        end
    end
         
    tau = tau + transpose(J_v_i) * F_i_rep;
    F_rep(:,i) = F_i_rep;

end

% motion plan doesn't converge if tau_rep normalized
% normalize tau
% if norm(tau) ~= 0
%     tau = tau/norm(tau);
% end

tau = transpose(tau);

% F_rep
% tau/norm(tau)

end