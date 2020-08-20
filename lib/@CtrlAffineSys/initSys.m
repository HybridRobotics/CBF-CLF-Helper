function initSys(obj, symbolic_s, symbolic_f, symbolic_g, symbolic_cbf, symbolic_clf)
    if isempty(symbolic_s) || isempty(symbolic_f) || isempty(symbolic_g)
        error('s, f, g is empty. Create a class function defineSystem and define your dynamics with symbolic expression.');
    end

    if ~isa(symbolic_f, 'sym')
        f_ = sym(symbolic_f);
    else
        f_ = symbolic_f;
    end
    if ~isa(symbolic_g, 'sym')
        g_ = sym(symbolic_g);
    else
        g_ = symbolic_g;
    end

    s = symbolic_s;
    % Setting state and input dimension.
    obj.sdim = size(s, 1);
    obj.udim = size(g_, 2);
    % Setting f and g (dynamics)
    obj.f = matlabFunction(f_, 'vars', {s});
    obj.g = matlabFunction(g_, 'vars', {s});            


    % Obtaining Lie derivatives of CBF.
    if ~isempty(symbolic_cbf)
        dcbf = simplify(jacobian(symbolic_cbf, symbolic_s));
        lf_cbf_ = dcbf * f_;
        lg_cbf_ = dcbf * g_;        
        obj.cbf = matlabFunction(symbolic_cbf, 'vars', {s});
        obj.lf_cbf = matlabFunction(lf_cbf_, 'vars', {s});
        obj.lg_cbf = matlabFunction(lg_cbf_, 'vars', {s});
    end

    % Obtaining Lie derivatives of CLF.    
    if ~isempty(symbolic_clf)
        dclf = simplify(jacobian(symbolic_clf, symbolic_s));
        lf_clf_ = dclf * f_;
        lg_clf_ = dclf * g_;
        obj.clf = matlabFunction(symbolic_clf, 'vars', {s});                       
        obj.lf_clf = matlabFunction(lf_clf_, 'vars', {s});
        obj.lg_clf = matlabFunction(lg_clf_, 'vars', {s});        
    end
end