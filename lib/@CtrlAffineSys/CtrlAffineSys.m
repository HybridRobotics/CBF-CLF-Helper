%% Author: Jason Choi (jason.choi@berkeley.edu)
classdef CtrlAffineSys < handle    
    %% Control-Affine Dynamic System Class.
    properties
        % State dimension
        sdim 
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
            [s, f, g] = obj.defineSystem(params);
            clf = obj.defineClf(params, s);
            cbf = obj.defineCbf(params, s);
            obj.initSys(s, f, g, cbf, clf);
        end
        
        %% Defining class function handles.
        function [s, f, g] = defineSystem(obj, params)
            % Outputs: s: symbolic state vector
            %          f: drift term, expressed symbolically wrt s.
            %          g: control vector fields, expressed symbolically wrt s.
            s = []; f = []; g = [];         
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
                
        function ds = dynamics(obj, t, s, u)
            % Inputs: t: time, s: state, u: control input
            % Output: ds: \dot(s)
            ds = obj.f(s) + obj.g(s) * u;
        end
    end
end

