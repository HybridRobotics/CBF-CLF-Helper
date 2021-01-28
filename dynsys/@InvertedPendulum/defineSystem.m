function [x, f, g] = defineSystem(obj, params)
    syms theta dtheta
    x = [theta; dtheta];
    l = params.l;    % [m]        length of pendulum
    m = params.m;    % [kg]       mass of pendulum
    g = params.g; % [m/s^2]    acceleration of gravity
    b = params.b; % [s*Nm/rad] friction coefficient

    f = [x(2); (-b*x(2) + m*g*l*sin(x(1))/2 ) / (m*l^2/3)];
    g = [0; -1 / (m*l^2/3)];
end