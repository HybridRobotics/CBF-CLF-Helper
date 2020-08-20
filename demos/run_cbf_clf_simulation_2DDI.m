close all;
clear all;
% Init state.
s0 = [0; 0; 4; 0];

% Target position
params.p_d = [10; 0];
% obstacle center
params.p_o = [5; 2];
% obstacle radius.
params.r_o = 2; 

dt = 0.02;
sim_t = 30;

params.cbf_gamma0 = 1;

params.u_max = 7;
params.u_min  = -7;

params.clf.rate = 0.7;
params.cbf.rate = 3;

params.weight.slack = 1;
params.weight.input = 5;

dynsys = DoubleIntegrator2D(params);

odeFun = @dynsys.dynamics;
controller = @dynsys.ctrlCbfClfQp;
odeSolver = @ode45;

total_k = ceil(sim_t / dt);
s = s0;
t = 0;   
% initialize traces.
ss = zeros(total_k, dynsys.sdim);
ts = zeros(total_k, 1);
us = zeros(total_k-1, dynsys.udim);
hs = zeros(total_k-1, 1);
Vs = zeros(total_k-1, 1);
ss(1, :) = s0';
ts(1) = t;
u_prev = [0;0];
for k = 1:total_k-1
    t
    % Determine control input.
    % dV_hat: analytic Vdot based on model.
    [u, slack, h, V] = controller(s);        
%     [u, slack, h, V] = controller(s, u_prev); % optimizing the difference between the previous timestep.       
    us(k, :) = u';
    hs(k) = h;
    Vs(k) = V;

    % Run one time step propagation.
    [ts_temp, ss_temp] = odeSolver(@(t, s) odeFun(t, s, u), [t t+dt], s);
    s = ss_temp(end, :)';

    ss(k+1, :) = s';
    ts(k+1) = ts_temp(end);
    u_prev = u;
    t = t + dt;
end

plot_results(ts, ss, us, params.p_o, params.r_o)


function plot_results(t, ss, us, p_o, r_o)

figure
subplot(5,1,1)
plot(t, ss(:,1))
xlabel('t')
ylabel('x [m]')

subplot(5,1,2)
plot(t, ss(:,2))
xlabel('t')
ylabel('v_x [m/s]')

subplot(5,1,3)
plot(t, ss(:,3))
xlabel('t')
ylabel('y [m]')

subplot(5,1,4)
plot(t, ss(:, 4))
xlabel('t')
ylabel('v_y [m/s]')

subplot(5,1,5)
plot(t, sqrt(ss(:, 2).^2 + ss(:, 4).^2))
xlabel('t')
ylabel('v [m/s]')


figure
subplot(2,1,1)
plot(t(1:end-1), us(:,1))
xlabel('t')
ylabel('a_x [m/s^2]')

subplot(2,1,2)
plot(t(1:end-1), us(:,2))
xlabel('t')
ylabel('a_y [m/s^2]')


lim_min = min(min(ss(:, 1)), min(ss(:, 3)));
lim_max = max(max(ss(:, 1)), max(ss(:, 3)));
lim_min = min([lim_min, p_o(1)-r_o, p_o(2)-r_o]);
lim_max = max([lim_max, p_o(1)+r_o, p_o(2)+r_o]);

figure
plot(ss(:, 1), ss(:, 3));
draw_circle(p_o, r_o);

xlim([lim_min, lim_max]);
ylim([lim_min, lim_max]);
xlabel('x [m]')
ylabel('y [m]')
end

function h = draw_circle(center,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = plot(xunit, yunit);
hold off
end