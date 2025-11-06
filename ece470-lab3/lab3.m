%% setup

% note: Lab 1 files are included
% forward.m
% inverse.m
% mypuma560.m

% robot setup
% DH = [theta_i, d_i, a_i, alpha_i]
DH = [0         76          0       pi/2;
      0     -23.65      43.23          0;
      0          0          0       pi/2;
      0      43.18          0      -pi/2;
      0          0          0       pi/2;
      0         20          0          0];
myrobot = mypuma560(DH);

%% 3.1
% see att.m for implementation of att function
% testing att function
H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
H1(1:3,4)=100*[-1; 3; 3;]/4; % This assigns the desired displacement to the hom.tsf.mtx.
q1 = inverse(H1,myrobot);
% This is the starting joint variable vector.
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);
q2(4) = q2(4) + 2*pi;
% This is the final joint variable vector
tau = att(q1,q2,myrobot)

%% 3.2
% see motionplan.m for implementation of motionplan function
% test using commands from lab handout
qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q);

%% 3.3 
% setup obstacles
setupobstacle;
% see rep.m for implementation of rep function

%% 3.3a test rep function with obs{1} --- cylinder
q3 = 0.9*q1+0.1*q2;
tau = rep(q3,myrobot,obs{1}) % This tests the torque for the cylinder obstacle

%% 3.3b test rep function with obs{6} --- sphere
q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot,obs{6})

%% 3.3c test motionplan with obstacles
% see motionplan.m for inclusion of repulsive forces
setupobstacle
hold on
axis([-100 100 -100 100 0 200])
view(-32,50)
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);
hold off


