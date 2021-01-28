%% Author: Jason Choi (jason.choi@berkeley.edu)
classdef CtrlAffineSys < handle    
    %% Control-Affine Dynamic System Class.
    properties
        % State dimension
        xdim 
        % Control input dimension
        udim
        % Model parameters as structure object, specific to the system.
        params 
        
        %% Control-Affine Dynamics: xdot = f(x) + g(x) u
        % Drift term in the dynamics as a function handle.
        f 
        % System vector fields in the dynamics as a function handle.
        g 
        
        %% CBF, CLF related properties.
        % Control Barrier Function as a function handle
        cbf 
        % Lie derivative (wrt f) of the CBF as a function handle
        lf_cbf 
        % Lie derivative (wrt g) of the CBF as a function handle
        lg_cbf 
        % Control Lyapunov Function as a function handle
        clf
        % Lie derivative (wrt f) of the CLF as a function handle
        lf_clf
        % Lie derivative (wrt g) of the CLF as a function handle
        lg_clf
    end
    
    methods
        function obj = CtrlAffineSys(params)
            if nargin < 1
                obj.params = [];
                disp("Warning: params argument is missing.")
            else
                obj.params = params;
            end
                            
            %% TODO: Add checking input constraint.
            %% TODO: Add existence of essential fields (e.g. params.weight.slack)
            [x, f, g] = obj.defineSystem(params);
            clf = obj.defineClf(params, x);
            cbf = obj.defineCbf(params, x);
            obj.initSys(x, f, g, cbf, clf);
        end
        
        %% Defining class function handles.
        function [x, f, g] = defineSystem(obj, params)
            % Outputs: x: symbolic state vector
            %          f: drift term, expressed symbolically wrt x.
            %          g: control vector fields, expressed symbolically wrt x.
            x = []; f = []; g = [];         
        end
        
        function clf = defineClf(obj, params, symbolic_state)
            % symbolic state: same symbolic state created in defineSystem
            % clf: CLF expressed symbolically wrt symbolic_state.
            clf = [];
        end
        
        function cbf = defineCbf(obj, params, symbolic_state)
            % symbolic state: same symbolic state created in defineSystem
            % cbf: CBF expressed symbolically wrt symbolic_state.
            cbf = [];
        end
                
        function dx = dynamics(obj, t, x, u)
            % Inputs: t: time, x: state, u: control input
            % Output: dx: \dot(x)
            dx = obj.f(x) + obj.g(x) * u;
        end
    end
end

