function myrobot = mykuka_search(d)
DH = [0  400.0    25.0  pi/2;
      0      0   315.0     0;
      0      0    35.0  pi/2;
      0  365.0       0 -pi/2;
      0      0       0  pi/2;
      0 161.44+d(2)  -296.23+d(1)     0;];
myrobot = SerialLink(DH);
end