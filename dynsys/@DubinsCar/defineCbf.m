function cbf = defineCbf(~, params, symbolic_s)
    s = symbolic_s;
    x = s(1); y = s(2); theta = s(3);

    v = params.v;
    xo = params.xo;
    yo = params.yo;
    d = params.d;

    distance = (x - xo)^2 + (y - yo)^2 - d^2;
    derivDistance = 2*(x-xo)*v*cos(theta) + 2*(y-yo)*sin(theta);
    cbf = derivDistance + params.cbf_gamma0 * distance; 
end