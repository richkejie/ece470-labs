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

% setup obstacles prepops
setupobstacle_lab4prep;

%% 3: Preparation: test forces
fprintf("Tau repulsive, for cylinder: \n");
q = [pi/10, pi/12, pi/6, pi/2, pi/2, -pi/6];
tau = rep(q, kuka_forces, prepobs{1}, 1);
disp(tau/norm(tau));
% rep doesn't normalize tau, because motion plan doesn't converge if
% normalized

% expecting: norm(tau) = [0.1795 0.9540 0.2353 -0.0344 -0.0344 0.0000]

% note: added eta parameter to rep function

%% 3: Preparation: test motion planning
fprintf("Testing motion planning with prep obstacles: \n")
p1 = [620 375 50];
p2 = [620 -375 50];
R = [0 0 1; 0 -1 0; 1 0 0];
H1 = [R p1'; zeros(1,3) 1];
H2 = [R p2'; zeros(1,3) 1];
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);
qref = motionplan(q1, q2, 0, 10, kuka_forces, prepobs, 0.01, 1, 0.013, 0.01);

% note: added eta_rep, alpha_att, alpha_rep parameters to motionplan
% function

% plot trajectory
hold on
plotobstacle(prepobs);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(kuka,q);
hold off

%% 4.1: Initial Motion Planning in Simulation
fprintf("Testing initial motion planning of kuka in simulation: \n")
z_grid = 45;
p0 = [370 -440 150];
p1 = [370 -440 z_grid];
p2 = [750 -220 225];
p3 = [620 350 225];

setupobstacle; % see setupobstacle.m for obstacles

eta = 1;
alpha_att = 0.01;
alpha_rep = 0.01;

R = [0 0 1; 0 -1 0; 1 0 0];

H0 = [R p0'; zeros(1,3) 1];
H1 = [R p1'; zeros(1,3) 1];
H2 = [R p2'; zeros(1,3) 1];
H3 = [R p3'; zeros(1,3) 1];

q_HOME = [0 1.5708 0 0 1.5708 0];

q0 = inverse_kuka(H0, kuka);
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);
q3 = inverse_kuka(H3, kuka);

t = linspace(0,10,300);

% uncomment the part you want to see

fprintf("HOME to q0: \n")
qref_0 = motionplan(q_HOME, q0, 0, 10, kuka_forces, obs, 0.01, eta, alpha_att, alpha_rep);
q_0_path = ppval(qref_0,t)';
% hold on
% plotobstacle(obs);
% plot(kuka,q_0_path);
% hold off

fprintf("q0 to q1: \n")
qref_1 = motionplan(q0, q1, 0, 10, kuka_forces, obs, 0.01, eta, alpha_att, alpha_rep);
q_1_path = ppval(qref_1,t)';
% hold on
% plotobstacle(obs);
% plot(kuka,q_1_path);
% hold off

fprintf("q1 to q2: \n")
qref_2 = motionplan(q1, q2, 0, 10, kuka_forces, obs, 0.01, eta, alpha_att, alpha_rep);
q_2_path = ppval(qref_2,t)';
% hold on
% plotobstacle(obs);
% plot(kuka,q_2_path);
% hold off

fprintf("q2 to q3: \n")
qref_3 = motionplan(q2, q3, 0, 10, kuka_forces, obs, 0.01, eta, alpha_att, alpha_rep);
q_3_path = ppval(qref_3,t)';
% hold on
% plotobstacle(obs);
% plot(kuka,q_3_path);
% hold off

%% 4.2 Commands to control KUKA arm and run motion planning
% make sure startConnection has been run

setHome(0.04)
setGripper(1) % open gripper

for i = 1:size(q_0_path,1)
    setAngles(q_0_path(i,:),0.04)
end

for i = 1:size(q_1_path,1)
    setAngles(q_1_path(i,:),0.04)
end

setGripper(0) % close

for i = 1:size(q_2_path,1)
    setAngles(q_2_path(i,:),0.04)
end

for i = 1:size(q_3_path,1)
    setAngles(q_3_path(i,:),0.04)
end

setGripper(1) % open

%% 4.3 Creative Motion Planning
%% simulation and calculation

% probably something like this:
% place multiple objects in a line

% same obstacles
% move EE over, then down to first object
% move EE up then over to dest and down to place
% repeat for 2 more objects


%% KUKA control commands
% make sure startConnection has been run



