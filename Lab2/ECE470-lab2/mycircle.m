X_workspace = zeros(3,100);
X_baseframe = zeros(3,100);
R = [0 0 1;
    0 -1 0;
    1 0 0];

for i=1:100
    X_workspace(1,i) = 620+50*sin(pi*i/50);
    X_workspace(2,i) = 50*cos(pi*i/50);
    X_workspace(3,i) = -1;

    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));

    H = [R X_baseframe(:,i);
        0 0 0 1];
    q = inverse_kuka(H,myrobot);
    setAngles(q,0.04);
end