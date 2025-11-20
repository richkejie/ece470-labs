clc
clear
close all

%% setup

% define parameters for DH
d1 = 400;
a1 = 25;
a2 = 315;
a3 = 35;
d4 = 365;
a6 = 156;
d6 = 161.44;

DH = [
      0     d1     a1     pi/2 ;
      0     0      a2     0    ;
      0     0      a3     pi/2 ;
      0     d4     0     -pi/2;
      0     0      0      pi/2 ;
      0     d6    -a6     0    ];

% kuka
kuka = mykuka(DH);

% kuka_forces
DH_forces = DH;
DH_forces(6,3) = 0;
kuka_forces = mykuka(DH_forces);

%% test forces
setupobstacle_lab4prep;
q = [pi/10, pi/12, pi/6, pi/2, pi/2, -pi/6];
tau = rep(q, kuka_forces, prepobs{1});

