data=xlsread('jug.xlsx');
xdata=550 + 10*data(:,1);
ydata=10*data(:,2);
zdata=-ones(length(data),1);

X_workspace = zeros(3,100);
X_baseframe = zeros(3,100);
R = [0 0 1;
    0 -1 0;
    1 0 0];

for i=1:length(data)
    X_workspace(1,i) = xdata(i,1);
    X_workspace(2,i) = ydata(i,1);
    X_workspace(3,i) = -2;

    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));

    H = [R X_baseframe(:,i);
        0 0 0 1];
    q = inverse_kuka(H,myrobot);
    setAngles(q,0.04);
end