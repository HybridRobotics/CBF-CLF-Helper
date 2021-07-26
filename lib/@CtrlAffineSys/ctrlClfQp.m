%% Author: Jason Choi (jason.choi@berkeley.edu)
function [u, slack, V, feas, comp_time] = ctrlClfQp(obj, x, u_ref, with_slack, verbose)
    %% Implementation of vanilla CLF-QP
    % Inputs:   x: state
    %           u_ref: reference control input
    %           with_slack: flag for relaxing (1: relax, 0: hard CLF constraint)
    %           verbose: flag for logging (1: print log, 0: run silently)
    % Outputs:  u: control input as a solution of the CLF-QP
    %           slack: slack variable for relaxation. (empty list when with_slack=0)
    %           V: Value of the CLF at the current state.
    %           feas: 1 if QP is feasible, 0 if infeasible. (Note: even
    %           when qp is infeasible, u is determined from quadprog.)
    %           comp_time: computation time to run the solver.

    if isempty(obj.clf)
        error('CLF is not defined so ctrlClfQp cannot be used. Create a class function [defineClf] and set up clf with symbolic expression.');
    end

    if nargin < 3 || isempty(u_ref)
        % If u_ref is given, CLF-QP minimizes the norm of u-u_ref        
        % Default reference control input is u.
        u_ref = zeros(obj.udim, 1);
    end
    if nargin < 4
        % Relaxing is activated in default condition.
        with_slack = 1;
    end
    if nargin < 5
        % Run QP without log in default condition.
        verbose = 0;
    end
    
    if size(u_ref, 1) ~= obj.udim
        error("Wrong size of u_ref, it should be (udim, 1) array.");
    end
    
    tstart = tic;
    V = obj.clf(x);
    % Lie derivatives of the CLF.
    LfV = obj.lf_clf(x);
    LgV = obj.lg_clf(x);

    %% Constraints : A[u; slack] <= b
    if with_slack
        % CLF constraint.
        A = [LgV, -1];
        b = -LfV-obj.params.clf.rate*V;                
        % Add input constraints if u_max or u_min exists.
        if isfield(obj.params, 'u_max')
            A = [A; eye(obj.udim), zeros(obj.udim, 1);];
            if size(obj.params.u_max, 1) == 1
                b = [b; obj.params.u_max * ones(obj.udim, 1)];
            elseif size(obj.params.u_max, 1) == obj.udim
                b = [b; obj.params.u_max];
            else
                error("params.u_max should be either a scalar value or an (udim, 1) array.")
            end
        end
        if isfield(obj.params, 'u_min')
            A = [A; -eye(obj.udim), zeros(obj.udim, 1);];
            if size(obj.params.u_min, 1) == 1
                b = [b; -obj.params.u_min * ones(obj.udim, 1)];
            elseif size(obj.params.u_min, 1) == obj.udim
                b = [b; -obj.params.u_min];
            else
                error("params.u_min should be either a scalar value or an (udim, 1) array")
            end
        end        
    else
        % CLF constraint.
        A = LgV;
        b = -LfV-obj.params.clf.rate*V;                
        % Add input constraints if u_max or u_min exists.
        if isfield(obj.params, 'u_max')
            A = [A; eye(obj.udim)];
            if size(obj.params.u_max, 1) == 1
                b = [b; obj.params.u_max * ones(obj.udim, 1)];
            elseif size(obj.params.u_max, 1) == obj.udim
                b = [b; obj.params.u_max];
            else
                error("params.u_max should be either a scalar value or an (udim, 1) array.")
            end
        end
        if isfield(obj.params, 'u_min')
            A = [A; -eye(obj.udim)];
            if size(obj.params.u_min, 1) == 1
                b = [b; -obj.params.u_min * ones(obj.udim, 1)];
            elseif size(obj.params.u_min, 1) == obj.udim
                b = [b; -obj.params.u_min];
            else
                error("params.u_min should be either a scalar value or an (udim, 1) array")
            end
        end
    end
    
    %% Cost
    if isfield(obj.params.weight, 'input')
        if size(obj.params.weight.input, 1) == 1 
            weight_input = obj.params.weight.input * eye(obj.udim);
        elseif all(size(obj.params.weight.input) == obj.udim)
            weight_input = obj.params.weight.input;
        else
            error("params.weight.input should be either a scalar value or an (udim, udim) array.")
        end
    else
        weight_input = eye(obj.udim);
    end
    
    if verbose
        options =  optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');
    else
        options =  optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'Display','off');
    end
    
    if with_slack         
        % cost = 0.5 [u' slack] H [u; slack] + f [u; slack]
        H = [weight_input, zeros(obj.udim, 1);
            zeros(1, obj.udim), obj.params.weight.slack];
        f_ = [-weight_input * u_ref; 0];
        [u_slack, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2            
            feas = 0;
            disp("Infeasible QP. Numerical error might have occured.");
            % Making up best-effort heuristic solution.
            u = zeros(obj.udim, 1);
            for i = 1:obj.udim
                u(i) = obj.params.u_min * (LgV(i) > 0) + obj.params.u_max * (LgV(i) <= 0);
            end
        else
            feas = 1;
            u = u_slack(1:obj.udim);
        end
        slack = u_slack(end);
    else
        H = weight_input;
        f_ = -weight_input * u_ref;
        [u, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2
            feas = 0;
            disp("Infeasible QP. CLF constraint is conflicting with input constraints.");
            % Making up best-effort heuristic solution.
            u = zeros(obj.udim, 1);
            for i = 1:obj.udim
                u(i) = obj.params.u_min * (LgV(i) > 0) + obj.params.u_max * (LgV(i) <= 0);
            end
        else
            feas = 1;
        end
        slack = [];
    end
    comp_time = toc(tstart);
end
