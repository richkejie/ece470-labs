
% 4.2: setup q matrix 
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
      0     -23.65      43.18          0;
      0          0          0       pi/2;
      0      43.18          0      -pi/2;
      0          0          0       pi/2;
      0         20          0          0];

myrobot = mypuma560(DH);

% 4.2: plot sample joint space trajectory
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






