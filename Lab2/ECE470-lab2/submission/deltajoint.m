function q = deltajoint(delta)
    % TODO 1/2: Add proper documentation for this function.

    kuka = mykuka_search(delta);

    %-------------------------- Calibration ----------------------------%
    % TODO 2/2: Fill in values Xi and Qi for i = {1, 2, 3}. Xi are 3 by 1
    % column vectors, while Qi are 1 by 6 row vectors.
    
    X1 = [710.74; 2.75; 25.23];
    X2 = [703.92; 89.89; 24.76];
    X3 = [625.54; 82.64; 26.55];
    Q1 = [     0.0037    0.6130    0.0002         0    1.0800         0];
    Q2 = [     0.1318    0.6070         0         0    1.0715         0];
    Q3 = [     0.1311    0.7622   -0.2332         0    1.0737         0];
    %-------------------------------------------------------------------%

    H1=forward_kuka(Q1, kuka);
    H2=forward_kuka(Q2, kuka);
    H3=forward_kuka(Q3, kuka);
    
    q=norm(H1(1:3,4)-X1)+norm(H2(1:3,4)-X2)+norm(H3(1:3,4)-X3);
end