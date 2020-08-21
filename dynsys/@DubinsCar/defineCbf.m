function cbf = defineCbf(~, params, symbolic_s)
    s = symbolic_s;
    x = s(1); y = s(2); theta = s(3);

    v = params.v;
    xo = params.xo;
    yo = params.yo;
    d = params.d;
    
    R_left = v / params.u_max;
    R_right = - v / params.u_min;

    % Left CBF
%     cbf = (x - xo - R_left * sin(theta))^2 + (y - yo + R_left * cos(theta))^2 - (R_left + d)^2;
    % Right CBF
%     cbf = (x - xo + R_right * sin(theta))^2 + (y - yo - R_right * cos(theta))^2 - (R_right + d)^2;

    distance = (x - xo)^2 + (y - yo)^2 - d^2;
    derivDistance = 2*(x-xo)*v*cos(theta) + 2*(y-yo)*sin(theta);
    cbf = derivDistance + params.cbf_gamma0 * distance; 
end