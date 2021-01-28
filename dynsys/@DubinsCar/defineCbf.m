function cbf = defineCbf(~, params, symbolic_state)
    x = symbolic_state;
    p_x = x(1); p_y = x(2); theta = x(3);

    v = params.v;
    xo = params.xo;
    yo = params.yo;
    d = params.d;

    distance = (p_x - xo)^2 + (p_y - yo)^2 - d^2;
    derivDistance = 2*(p_x-xo)*v*cos(theta) + 2*(p_y-yo)*sin(theta);
    cbf = derivDistance + params.cbf_gamma0 * distance; 
end