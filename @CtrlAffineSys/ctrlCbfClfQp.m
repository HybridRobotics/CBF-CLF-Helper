function [u, slack, B, V] = ctrlCbfClfQp(obj, s, u_ref, with_slack)
    if isempty(obj.clf)
        error('CLF is not defined so ctrlCbfClfQp cannot be used. Create a class function defineClf and set up clf with symbolic expression.');
    end
    if isempty(obj.cbf)
        error('CBF is not defined so ctrlCbfClfQp cannot be used. Create a class function defineCbf and set up cbf with symbolic expression.');
    end
    
    if nargin < 3
        u_ref = zeros(obj.udim, 1);
    end
    if nargin < 4
        with_slack = 1;
    end
    
    if size(u_ref, 1) ~= obj.udim
        error("Wrong size of u_ref, it should be (udim, 1) array.");
    end                
    B = obj.cbf(s);
    LfB = obj.lf_cbf(s);
    LgB = obj.lg_cbf(s);
    V = obj.clf(s);
    LfV = obj.lf_clf(s);
    LgV = obj.lg_clf(s);

    %% Constraints : A[u; slack] <= b
    %% TODO: generalize the code to higher relative degree.
    if with_slack
        A = [LgV, -1;
        -LgB, 0];
        b = [-LfV - obj.params.clf.rate * V;
            LfB + obj.params.cbf.rate * B];                
        %% Add input constraints if u_max or u_min exists.
        if isfield(obj.params, 'u_max')
            A = [A; ones(obj.udim), zeros(obj.udim, 1);];
            if size(obj.params.u_max, 1) == 1
                b = [b; obj.params.u_max * ones(obj.udim, 1)];
            elseif size(obj.params.u_max, 1) == obj.udim
                b = [b; obj.params.u_max];
            else
                error("params.u_max should be either a scalar value or an (udim, 1) array.")
            end
        end
        if isfield(obj.params, 'u_min')
            A = [A; -ones(obj.udim), zeros(obj.udim, 1);];
            if size(obj.params.u_min, 1) == 1
                b = [b; -obj.params.u_min * ones(obj.udim, 1)];
            elseif size(obj.params.u_min, 1) == obj.udim
                b = [b; -obj.params.u_min];
            else
                error("params.u_min should be either a scalar value or an (udim, 1) array")
            end
        end        
    else
        A = [LgV; -LgB];
        b = [-LfV - obj.params.clf.rate * V;
            LfB + obj.params.cbf.rate * B];                
        %% Add input constraints if u_max or u_min exists.
        if isfield(obj.params, 'u_max')
            A = [A; ones(obj.udim)];
            if size(obj.params.u_max, 1) == 1
                b = [b; obj.params.u_max * ones(obj.udim, 1)];
            elseif size(obj.params.u_max, 1) == obj.udim
                b = [b; obj.params.u_max];
            else
                error("params.u_max should be either a scalar value or an (udim, 1) array.")
            end
        end
        if isfield(obj.params, 'u_min')
            A = [A; -ones(obj.udim)];
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
    
    if with_slack         
        % cost = 0.5 [u' slack] H [u; slack] + f [u; slack]
        H = [weight_input, zeros(obj.udim, 1);
            zeros(1, obj.udim), obj.params.weight.slack];
        f_ = [-weight_input * u_ref; 0];
        [u_slack, ~, exitflag, ~] = quadprog(H, f_, A, b);
        if exitflag == -2
            disp("Infeasible QP. CBF constraint is conflicting with input constraints.");
        end

        u = u_slack(1:obj.udim);
        slack = u_slack(end);
    else
        H = weight_input;
        f_ = -weight_input * u_ref;
        [u, ~, exitflag, ~] = quadprog(H, f_, A, b);
        if exitflag == -2
            disp("Infeasible QP. CBF constraint is conflicting with input constraints.");
        end
        slack = [];
    end
end