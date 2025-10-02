
% 4.2: setup q matrix and plot sample joint space trajectory
q = zeros(6,200);
q(1,:) = linspace(0,pi,200);
q(2,:) = linspace(0,pi/2,200);
q(3,:) = linspace(0,pi,200);
q(4,:) = linspace(pi/4,3*pi/4,200);
q(5,:) = linspace(-pi/3,pi/3,200);
q(6,:) = linspace(0,2*pi,200);
q = transpose(q); % want 200x6

% DH = [theta_i, d_i, a_i, alpha_i]
DH = [0         76          0       pi/2;
      0     -23.65      43.23          0;
      0          0          0       pi/2;
      0      43.18          0      -pi/2;
      0          0          0       pi/2;
      0         20          0          0];
myrobot = mypuma560(DH);
% see lab1-4.2.fig
% plot(myrobot,q);

% 4.3: forward kinematics
o = zeros(200,3); % position of end effector
q_size = size(q);
num_rows = q_size(1);
for i=1:num_rows
    H = forward(transpose(q(i,:)),myrobot);
    o(i,:) = H(1:3,4);
end
% plot trajectory of end effector - matches predicted red path
% see lab1-4.3.fig
% plot3(o(:,1),o(:,2),o(:,3), 'r');
% hold on;
% plot(myrobot,q);

% 4.4: inverse kinematics
% test inverse function is correct
H_test = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
q_expected = [-0.0331 -1.0667 1.0283 3.1416 3.1032 0.8185];
q_test = inverse(H_test,myrobot);
% test picking up an object
% 1) d: sequence of steps of x,y,z from (10,23,15) to (30,30,100)
d = zeros(3,100);
d(1,:) = linspace(10,30,100);
d(2,:) = linspace(23,30,100);
d(3,:) = linspace(15,100,100);
d = transpose(d);
% 2) R_z_pi/4
R_z = [ cos(pi/4)    -sin(pi/4)  0;
        sin(pi/4)     cos(pi/4)  0;
                0             0  1];
% 3) solve inverse
for i=1:100
    H_i = [  R_z    transpose(d(i,:));
            0 0 0                   1];
    q(i,:) = inverse(H_i,myrobot);
end
% plot trajectory
plot3(d(:,1),d(:,2),d(:,3), 'r');
xlim([-100,100])
ylim([-100,100])
zlim([-50,300])
hold on;
plot(myrobot,q);





