function [s, f, g] = defineSystem(obj, params)
    syms theta dtheta
    s = [theta; dtheta];
    l = params.l;    % [m]        length of pendulum
    m = params.m;    % [kg]       mass of pendulum
    g = params.g; % [m/s^2]    acceleration of gravity
    b = params.b; % [s*Nm/rad] friction coefficient

    f = [s(2); (-b*s(2) + m*g*l*sin(s(1))/2 ) / (m*l^2/3)];
    g = [0; -1 / (m*l^2/3)];
end