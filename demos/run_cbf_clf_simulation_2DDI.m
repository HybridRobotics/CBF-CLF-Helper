close all;
clear all;
% Init state.
x0 = [0; 0; 4; 0];

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
x = x0;
t = 0;   
% initialize traces.
xs = zeros(total_k, dynsys.xdim);
ts = zeros(total_k, 1);
us = zeros(total_k-1, dynsys.udim);
hs = zeros(total_k-1, 1);
Vs = zeros(total_k-1, 1);
xs(1, :) = x0';
ts(1) = t;
u_prev = [0;0];
for k = 1:total_k-1
    t
    % Determine control input.
    % dV_hat: analytic Vdot based on model.
    [u, slack, h, V] = controller(x);        
%     [u, slack, h, V] = controller(s, u_prev); % optimizing the difference between the previous timestep.       
    us(k, :) = u';
    hs(k) = h;
    Vs(k) = V;

    % Run one time step propagation.
    [ts_temp, xs_temp] = odeSolver(@(t, s) odeFun(t, s, u), [t t+dt], x);
    x = xs_temp(end, :)';

    xs(k+1, :) = x';
    ts(k+1) = ts_temp(end);
    u_prev = u;
    t = t + dt;
end

plot_results(ts, xs, us, params.p_o, params.r_o)


function plot_results(t, xs, us, p_o, r_o)

figure
subplot(5,1,1)
plot(t, xs(:,1))
xlabel('t')
ylabel('x [m]')

subplot(5,1,2)
plot(t, xs(:,2))
xlabel('t')
ylabel('v_x [m/s]')

subplot(5,1,3)
plot(t, xs(:,3))
xlabel('t')
ylabel('y [m]')

subplot(5,1,4)
plot(t, xs(:, 4))
xlabel('t')
ylabel('v_y [m/s]')

subplot(5,1,5)
plot(t, sqrt(xs(:, 2).^2 + xs(:, 4).^2))
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


lim_min = min(min(xs(:, 1)), min(xs(:, 3)));
lim_max = max(max(xs(:, 1)), max(xs(:, 3)));
lim_min = min([lim_min, p_o(1)-r_o, p_o(2)-r_o]);
lim_max = max([lim_max, p_o(1)+r_o, p_o(2)+r_o]);

figure
plot(xs(:, 1), xs(:, 3));
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