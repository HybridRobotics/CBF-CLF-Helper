function clf = defineClf(obj, params, symbolic_state)
    v = symbolic_state(2);
    vd = params.vd; % desired velocity.           
    
    clf = (v - vd)^2;
end