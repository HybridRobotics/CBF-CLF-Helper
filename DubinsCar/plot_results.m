function plot_results(t, ss, us, hs, left_actives, p_o, r_o)

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
plot(t(1:end-1), left_actives)
xlabel('t')
ylabel('active cbf constraint (1: left, 0: right)');

figure
plot(t(1:end-1), hs)
xlabel('t')
ylabel('cbf h(s)');

function h = draw_circle(center,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = plot(xunit, yunit);
hold off