function cbf = defineCbf(obj, params, symbolic_state)
    v = symbolic_state(2);
    z = symbolic_state(3);

    v0 = params.v0;
    T = params.T;
    cd = params.cd;
    
    cbf = z - T * v - 0.5  * (v0-v)^2 / (cd * params.g);
end