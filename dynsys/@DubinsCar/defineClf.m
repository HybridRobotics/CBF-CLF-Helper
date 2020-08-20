function clf = defineClf(~, params, symbolic_s)
    x = symbolic_s(1);
    y = symbolic_s(2);
    theta = symbolic_s(3);
    clf = (cos(theta).*(y-params.yd)-sin(theta).*(x-params.xd)).^2;
end