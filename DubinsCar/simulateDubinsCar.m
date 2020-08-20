close all
clear
%% main file for simulation

settingsDubinsCar;

odeFun = @dynamics;
controller = @(t, s)cbf_clf_ctrl(t, s, params);
odeSolver = @ode45;

total_k = ceil(sim_t / dt);
s = s0;
t = 0;   
% initialize traces.
ss = zeros(total_k, 3);
ts = zeros(total_k, 1);
us = zeros(total_k-1, 1);
hs = zeros(total_k-1, 1);
left_actives = zeros(total_k-1, 1);
ss(1, :) = s0';
ts(1) = t;
for k = 1:total_k-1
    t
    % Determine control input.
    % dV_hat: analytic Vdot based on model.
    [u, left_active, h] = controller(t, s);        
    us(k, :) = u';
    left_actives(k) = left_active;
    hs(k) = h;

    % Run one time step propagation.
    [ts_temp, ss_temp] = odeSolver(@(t, s) odeFun(t, s, u, params), [t t+dt], s);
    s = ss_temp(end, :)';

    ss(k+1, :) = s';
    ts(k+1) = ts_temp(end);
    t = t + dt;
end

plot_results(ts, ss, us, hs, left_actives, [params.xo;params.yo], params.d)
