function clf = defineClf(~, params, symbolic_state)
    x = symbolic_state;
    I = params.m * params.l^2 / 3;
    c_bar = params.m*params.g*params.l/(2*I);
    b_bar = params.b/I;
    A = [0, 1; 
        c_bar-params.Kp/I, -b_bar-params.Kd/I]; % Linearized Dynamics.
    Q = params.clf.rate * eye(size(A,1));
    P = lyap(A', Q); % Cost Matrix for quadratic CLF. (V = e'*P*e)
    clf = x' * P * x;
end

