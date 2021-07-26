%% Author: Jason Choi (jason.choi@berkeley.edu)
function [u, slack, B, V, feas, comp_time] = ctrlCbfClfQp(obj, x, u_ref, with_slack, verbose)
    %% Implementation of vanilla CBF-CLF-QP
    % Inputs:   x: state
    %           u_ref: reference control input
    %           with_slack: flag for relaxing the clf constraint(1: relax, 0: hard-constraint)
    %           verbose: flag for logging (1: print log, 0: run silently)
    % Outputs:  u: control input as a solution of the CBF-CLF-QP
    %           slack: slack variable for relaxation. (empty list when with_slack=0)
    %           B: Value of the CBF at current state.
    %           V: Value of the CLF at current state.
    %           feas: 1 if QP is feasible, 0 if infeasible. (Note: even
    %           when qp is infeasible, u is determined from quadprog.)
    %           comp_time: computation time to run the solver.
    if isempty(obj.clf)
        error('CLF is not defined so ctrlCbfClfQp cannot be used. Create a class function [defineClf] and set up clf with symbolic expression.');
    end
    if isempty(obj.cbf)
        error('CBF is not defined so ctrlCbfClfQp cannot be used. Create a class function [defineCbf] and set up cbf with symbolic expression.');
    end
        
    if nargin < 3
        u_ref = zeros(obj.udim, 1);
    end
    if nargin < 4
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
    LfV = obj.lf_clf(x);
    LgV = obj.lg_clf(x);

    B = obj.cbf(x);
    LfB = obj.lf_cbf(x);
    LgB = obj.lg_cbf(x);
        
    %% Constraints: A[u; slack] <= b
    if with_slack
        % CLF and CBF constraints.
        A = [LgV, -1;
        -LgB, 0];
        b = [-LfV - obj.params.clf.rate * V;
            LfB + obj.params.cbf.rate * B];                
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
        % CLF and CBF constraints.
        A = [LgV; -LgB];
        b = [-LfV - obj.params.clf.rate * V;
            LfB + obj.params.cbf.rate * B];                
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
        options =  optimset('Display','notify');
    else
        options =  optimset('Display','off');
    end
    if with_slack         
        % cost = 0.5 [u' slack] H [u; slack] + f [u; slack]
        H = [weight_input, zeros(obj.udim, 1);
            zeros(1, obj.udim), obj.params.weight.slack];
        f_ = [-weight_input * u_ref; 0];
        [u_slack, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2
            feas = 0;
            disp("Infeasible QP. CBF constraint is conflicting with input constraints.");
        else
            feas = 1;
        end
        u = u_slack(1:obj.udim);
        slack = u_slack(end);
    else
        % cost = 0.5 u' H u + f u
        H = weight_input;
        f_ = -weight_input * u_ref;
        [u, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2
            feas = 0;
            disp("Infeasible QP. CBF constraint is conflicting with input constraints.");
        else
            feas = 1;
        end
        slack = [];
    end
    comp_time = toc(tstart);
end