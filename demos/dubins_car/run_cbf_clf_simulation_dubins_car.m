dt = 0.02;
sim_t = 20;
s0 = [0;5;0];

load_params_dubins_car;
dubins = DubinsCar(params);

odeFun = @dubins.dynamics;
controller = @dubins.ctrlCbfClfQp;
odeSolver = @ode45;

total_k = ceil(sim_t / dt);
s = s0;
t = 0;   
% initialize traces.
ss = zeros(total_k, dubins.sdim);
ts = zeros(total_k, 1);
us = zeros(total_k-1, 1);
Vs = zeros(total_k-1, 1);
hs = zeros(total_k-1, 1);
ss(1, :) = s0';
ts(1) = t;
for k = 1:total_k-1
    t
    % Determine control input.
    % dV_hat: analytic Vdot based on model.
    [u, slack, h, V] = controller(s);        
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

plot_results(ts, ss, us, hs, [params.xo;params.yo], params.d)

function plot_results(t, ss, us, hs, p_o, r_o)

figure
subplot(3,1,1)
plot(t, ss(:,1))
xlabel('t')
ylabel('x [m]')

subplot(3,1,2)
plot(t, ss(:,2))
xlabel('t')
ylabel('y [m]')

subplot(3,1,3)
plot(t, ss(:,3))
xlabel('t')
ylabel('theta [rad]')


figure
plot(t(1:end-1), us)
xlabel('t')
ylabel('u [rad/s]')


lim_min = min(min(ss(:, 1)), min(ss(:, 2)));
lim_max = max(max(ss(:, 1)), max(ss(:, 2)));
lim_min = min([lim_min, p_o(1)-r_o, p_o(2)-r_o]);
lim_max = max([lim_max, p_o(1)+r_o, p_o(2)+r_o]);

figure
plot(ss(:, 1), ss(:, 2));
draw_circle(p_o, r_o);

xlim([lim_min, lim_max]);
ylim([lim_min, lim_max]);
xlabel('x [m]')
ylabel('y [m]')

figure
plot(t(1:end-1), hs)
xlabel('t')
ylabel('cbf h(s)');

end

function h = draw_circle(center,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = plot(xunit, yunit);
hold off

end