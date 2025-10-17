%% 4.2
% q1:     0.0037    0.6130    0.0002         0    1.0800         0
% x1:   [710.74; 2.75; 25.23]
% q2:     0.1318    0.6070         0         0    1.0715         0
% x2:   [703.92; 89.89; 24.76];
% q3:     0.1311    0.7622   -0.2332         0    1.0737         0
% x3:   [625.54; 82.64; 26.55]

delta = fminunc(@deltajoint, [0 0]);
myrobot = mykuka_search(delta);

R_6_0 = [0 0 1;
         0 -1 0;
         1 0 0];
o_6_0 = [710.74; 2.75; 25.23];
H = [R_6_0 o_6_0;
    0 0 0 1];

q = inverse_kuka(H,myrobot);

%% 4.3
p_workspace = [600; 100; 10];
p_baseframe = FrameTransformation(p_workspace)
%p_baseframe =
%
%  600.0663
%  100.0213
%   36.9938
R = [0 0 1; 0 -1 0; 1 0 0];
H = [R p_baseframe; zeros(1,3) 1];
q = inverse_kuka(H,myrobot)
%q =
%
%    0.2207    0.8221   -0.2828    0.2557    1.0459   -0.1303


%% 4.4
% line segment: see mysegment.m, imgs/line.jpg
% cicle: see mycircle.m, imgs/circle.jpg
% jug: see jug.m
% our picture (a goose): see custom_drawing.m, goose1.csv, vids/goose.mp4
