clear all; close all;

dt = 0.02;
sim_t = 5;

%% Model Parameters
params.l = 1;    % [m]        length of pendulum
params.m = 1;    % [kg]       mass of pendulum
params.g = 9.81; % [m/s^2]    acceleration of gravity
params.b = 0.01; % [s*Nm/rad] friction coefficient

params.u_max = 7;
params.u_min = -params.u_max;

params.I = params.m*params.l^2/3; 

% Assumed feedback gains, used to construct a CLF.
params.Kp=6;
params.Kd=5;

params.clf.rate = 3;
params.weight.slack = 100000;

x0 = [0.76; 0.05];

ip_sys = InvertedPendulum(params);

odeFun = @ip_sys.dynamics;
controller = @ip_sys.ctrlClfQp;
odeSolver = @ode45;

total_k = ceil(sim_t / dt);
x = x0;
t = 0;   
% initialize traces.
xs = zeros(total_k, ip_sys.xdim);
ts = zeros(total_k, 1);
us = zeros(total_k-1, 1);
Vs = zeros(total_k-1, 1);
xs(1, :) = x0';
ts(1) = t;
for k = 1:total_k-1
    t
    % Determine control input.
    % dV_hat: analytic Vdot based on model.
    [u, slack, V] = controller(x);        
    us(k, :) = u';
    Vs(k) = V;

    % Run one time step propagation.
    [ts_temp, xs_temp] = odeSolver(@(t, s) odeFun(t, s, u), [t t+dt], x);
    x = xs_temp(end, :)';

    xs(k+1, :) = x';
    ts(k+1) = ts_temp(end);
    t = t + dt;
end

figure;
title('Inverted Pendulum: CLF-QP States');
subplot(2, 1, 1);
plot(ts, 180 * xs(:, 1)/pi);
xlabel("t (sec)"); ylabel("theta (deg)");

subplot(2, 1, 2);
plot(ts, 180 * xs(:, 2)/pi);
xlabel("t (sec)"); ylabel("dtheta (deg/s)");

figure;
plot(ts(1:end-1), us); hold on;
plot(ts(1:end-1), params.u_max*ones(size(ts, 1)-1, 1), 'k--');
plot(ts(1:end-1), params.u_min*ones(size(ts, 1)-1, 1), 'k--');
xlabel("t (sec)"); ylabel("u (N.m)");