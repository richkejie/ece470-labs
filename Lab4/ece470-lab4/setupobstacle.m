obs{1}.R = 100;
obs{1}.c = [620; 0]; % lies on the z = 0 plane
obs{1}.rho0 = 1000;
obs{1}.h = 572;
obs{1}.type = 'cyl';

cyl1 = [620 0 50];
H_cyl1 = [R cyl1'; zeros(1,3) 1];
q_cyl1 = inverse_kuka(H_cyl1, kuka);

obs{2}.R = 100;
obs{2}.c = [620; -440]; % lies on the z = 0 plane
obs{2}.rho0 = 150;
obs{2}.h = 572;
obs{2}.type = 'cyl';

cyl2 = [620 -440 50];
H_cyl2 = [R cyl2'; zeros(1,3) 1];
q_cyl2 = inverse_kuka(H_cyl2, kuka);

% z_plane = 46;
z_plane = 32;

obs{3}.p = [0; 0; z_plane];
obs{3}.n = [0; 0; 1];
obs{3}.rho0 = 150;
obs{3}.type = 'plane'; 