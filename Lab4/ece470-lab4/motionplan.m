function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol, eta_rep, alpha_att, alpha_rep)
    % qref is PWCP
    % q0 is col vector of initial actual joint angles
    % q2 is col vector of final joint angles
    % t1 is start time of PWCP
    % t2 is finish time of PWCP
    % obs is obstacle structure
    % tol is tolerance for termination

    q = q0;
    q_k = q0;

    % alpha_att = 0.013;
    % alpha_rep = 0.01;

    % terminate when norms of q and q2 within tol
    while (norm(q(end,1:5)-q2(1:5)) >= tol)
        % iterative step: q_k+1 = q_k + alpha_k * tau(q_k)/norm(tau(q_k))
        alpha = 0.01;
        tau_att = att(q(end, 1:6),q2,myrobot);
        tau_rep = zeros(1,6);
        for i = 1:size(obs)
            tau_rep = tau_rep + rep(q(end, 1:6),myrobot,obs{i}, eta_rep);
        end

        tau = alpha_att*tau_att + alpha_rep*tau_rep;

        q(end+1, 1:6) = q(end, 1:6) + tau;
        
        error = norm(q(end,1:5)-q2(1:5));

        step = size(q,1)-1;
        if mod(step,100) == 0 
            fprintf("Step " + (step) + ", error of " + error + "\n");
            fprintf("\n\n");
        end
    end
    fprintf("Step " + (step) + ", error of " + error + "\n");
    fprintf("\n\n");

    q(:,6) = linspace(q0(6), q2(6), size(q,1));

    % get spline
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q');

end