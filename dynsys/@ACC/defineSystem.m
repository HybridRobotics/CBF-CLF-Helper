function [x, f, g] = defineSystem(~, params)
    syms p v z % states
    x = [p; v; z];

    f0 = params.f0;
    f1 = params.f1;
    f2 = params.f2;
    v0 = params.v0;
    m = params.m;
    Fr = f0 + f1 * v + f2 * v^2;

    % Dynamics
    f = [v; -Fr/m; v0-v];
    g = [0; 1/m; 0]; 
end