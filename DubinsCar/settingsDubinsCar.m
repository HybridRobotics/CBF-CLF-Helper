dt = 0.02;
sim_t = 10;
s0 = [0;5;0];

params.V = 1; % velocity
params.u_max = 3; % max yaw rate (left)
params.u_min = -3; % min yaw rate (right)

params.R_left = params.V / params.u_max;
params.R_right = - params.V / params.u_min;

params.xo = 5;
params.yo = 4;
params.d = 2;

params.xd = 12;
params.yd = 0;

params.clf.lambda = 0.5;
params.clf.slack_weight = 10;

params.cbf.gamma = 1;
