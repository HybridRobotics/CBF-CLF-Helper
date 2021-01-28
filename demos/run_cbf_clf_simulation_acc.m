clear all; close all;

dt = 0.02;
sim_t = 20;
x0 = [0; 20; 100];

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

params.clf.rate = 5;
params.cbf.rate = 5;


params.weight.input = 2/params.m^2;
params.weight.slack = 2e-2;

%%
accSys = ACC(params);

odeFun = @accSys.dynamics;
controller = @accSys.ctrlCbfClfQp;
odeSolver = @ode45;

total_k = ceil(sim_t / dt);
x = x0;
t = 0;   
% initialize traces.
xs = zeros(total_k, 3);
ts = zeros(total_k, 1);
us = zeros(total_k-1, 1);
slacks = zeros(total_k-1, 1);
hs = zeros(total_k-1, 1);
Vs = zeros(total_k-1, 1);
xs(1, :) = x0';
ts(1) = t;
for k = 1:total_k-1
    t
    Fr = accSys.getFr(x);
    % Determine control input.
    [u, slack, h, V] = controller(x, Fr);        
    us(k, :) = u';
    slacks(k, :) = slack;
    hs(k) = h;
    Vs(k) = V;

    % Run one time step propagation.
    [ts_temp, xs_temp] = odeSolver(@(t, s) odeFun(t, s, u), [t t+dt], x);
    x = xs_temp(end, :)';

    xs(k+1, :) = x';
    ts(k+1) = ts_temp(end);
    t = t + dt;
end

plot_results(ts, xs, us, slacks, hs, Vs, params)


function plot_results(ts, xs, us, slacks, hs, Vs, params)
    fig_sz = [10 15]; 
    plot_pos = [0 0 10 15];
    yellow = [0.998, 0.875, 0.529];
    blue = [0.106, 0.588, 0.953];
    navy = [0.063, 0.075, 0.227];
    magenta = [0.937, 0.004, 0.584];
    orange = [0.965, 0.529, 0.255];
    grey = 0.01 *[19.6, 18.8, 19.2];
    
    figure(1);
    subplot(6,1,1);
    p = plot(ts, xs(:, 2));
    p.Color = blue;
    p.LineWidth = 1.5;
    hold on;
    plot(ts, params.vd*ones(size(ts, 1), 1), 'k--');
    ylabel("v (m/s)");
    title("State - Velocity");
    set(gca,'FontSize',14);
    grid on;    
    

    subplot(6,1,2);
    p = plot(ts, xs(:, 3));
    p.Color = magenta;
    p.LineWidth = 1.5;
    ylabel("z (m)");
    title("State - Distance to lead vehicle");
    set(gca, 'FontSize', 14);
    grid on;    
    
    subplot(6,1,3);
    p = plot(ts(1:end-1), us); hold on;
    p.Color = orange;
    p.LineWidth = 1.5;
    plot(ts(1:end-1), params.u_max*ones(size(ts, 1)-1, 1), 'k--');
    plot(ts(1:end-1), params.u_min*ones(size(ts, 1)-1, 1), 'k--');
    ylabel("u(N)");
    title("Control Input - Wheel Force");    
    set(gca, 'FontSize', 14);
    grid on;    

    subplot(6,1,4);
    p = plot(ts(1:end-1), slacks); hold on;
    p.Color = magenta;
    p.LineWidth = 1.5;
    ylabel("slack");
    title("Slack variable");        
    set(gca, 'FontSize', 14);
    grid on;    

    
    subplot(6,1,5);
    p = plot(ts(1:end-1), hs);
    p.Color = navy;
    p.LineWidth = 1.5;
    ylabel("CBF (h(x))");
    title("CBF");    
    set(gca, 'FontSize', 14);
        grid on;    

    subplot(6,1,6);
    set(gca, 'FontSize', 14);
    p = plot(ts(1:end-1), Vs);
    p.Color = navy;
    p.LineWidth = 1.5;
    xlabel("t(s)");
    ylabel("CLF (V(x))");
    title("CLF");    
    set(gca, 'FontSize', 14);
    grid on;    

    set(gcf, 'PaperSize', fig_sz);
    set(gcf, 'PaperPosition', plot_pos);
end
