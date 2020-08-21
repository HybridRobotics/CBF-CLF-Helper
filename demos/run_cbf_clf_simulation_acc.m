clear all; close all;

dt = 0.02;
sim_t = 20;
s0 = [0; 20; 100];

%% Parameters are from 
% Aaron Ames et al. Control Barrier Function based Quadratic Programs 
% with Application to Adaptive Cruise Control, CDC 2014, Table 1.

params.v0 = 14;
params.vd = 24;
params.m  = 1650;
params.g = 9.81;
params.f0 = 0.1;
params.f1 = 5;
params.f2 = 0.25;
params.ca = 0.3;
params.cd = 0.3;
params.T = 1.8;

params.u_max = params.ca * params.m * params.g;
params.u_min  = -params.cd * params.m * params.g;

params.clf.rate = 10;
params.cbf.rate = 1;

params.weight.input = 2/params.m^2;
params.weight.slack = 2e-2;

%%
accSys = ACC(params);

odeFun = @accSys.dynamics;
controller = @accSys.ctrlCbfClfQp;
odeSolver = @ode45;

total_k = ceil(sim_t / dt);
s = s0;
t = 0;   
% initialize traces.
ss = zeros(total_k, 3);
ts = zeros(total_k, 1);
us = zeros(total_k-1, 1);
hs = zeros(total_k-1, 1);
Vs = zeros(total_k-1, 1);
ss(1, :) = s0';
ts(1) = t;
for k = 1:total_k-1
    t
    Fr = accSys.getFr(s);
    % Determine control input.
    [u, slack, h, V] = controller(s, Fr);        
    us(k, :) = u';
    hs(k) = h;
    Vs(k) = V;

    % Run one time step propagation.
    [ts_temp, ss_temp] = odeSolver(@(t, s) odeFun(t, s, u), [t t+dt], s);
    s = ss_temp(end, :)';

    ss(k+1, :) = s';
    ts(k+1) = ts_temp(end);
    t = t + dt;
end

plot_results(ts, ss, us, hs, Vs, params)


function plot_results(ts, ss, us, hs, Vs, params)
    figure;
    subplot(4,1,1);
    plot(ts, ss(:, 2));
    xlabel("t(s)");
    ylabel("v (m/s)");
    
    subplot(4,1,2);
    plot(ts(1:end-1), us); hold on;
    plot(ts(1:end-1), params.u_max*ones(size(ts, 1)-1, 1), 'k--');
    plot(ts(1:end-1), params.u_min*ones(size(ts, 1)-1, 1), 'k--');
    xlabel("t(s)");
    ylabel("u(N)");
    
    subplot(4,1,3);
    plot(ts(1:end-1), hs);
    xlabel("t(s)");
    ylabel("CBF (h(x))");
    
    subplot(4,1,4);
    plot(ts(1:end-1), Vs);
    xlabel("t(s)");
    ylabel("CLF (V(x))");
end
