%% DH table
DH = [0  400.0    25.0  pi/2;
      0      0   315.0     0;
      0      0    35.0  pi/2;
      0  365.0       0 -pi/2;
      0      0       0  pi/2;
      0 161.44  -296.23     0;];

%% setup kuka robot
kuka = mykuka(DH);

%% test
q = [pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4];
H = forward_kuka(q,kuka)
q_check = inverse_kuka(H,kuka)