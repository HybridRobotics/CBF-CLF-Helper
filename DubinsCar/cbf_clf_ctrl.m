function [u, left_active, h] = cbf_clf_ctrl(~, s, params)

[h, Lfh, Lgh, left_active] = eval_cbf(s, params);
disp(h)
[lyap, LgLyap] = eval_clf(s, params);

%% Using solver.
A_constraint = [LgLyap, -1;
    -Lgh, 0;
    1, 0;
    -1, 0];
b_constraint = [-params.clf.lambda*lyap;
    Lfh + params.cbf.gamma*h;
    params.u_max;
    -params.u_min];

H = [1, 0; 0, params.clf.slack_weight];
f = [0; 0];

[u, ~, exitflag, ~] = quadprog(H, f, A_constraint, b_constraint)

if exitflag == -2
    disp("infeasible");
end    
u = u(1);

%% By hand. Only CBF
% u_cbf = -(Lfh + params.cbf.gamma * h)/Lgh;
% if Lgh < 0
%     if u_cbf < params.u_min
%         disp("infeasible solution.")
%         u = params.u_min;
%         return
%     end
%     u_max = u_cbf;
%     u_min = params.u_min;
% else
%     if u_cbf > params.u_max
%         disp("infeasible solution.")
%         u = params.u_max;
%         return
%     end
%     u_min = u_cbf;
%     u_max = params.u_max;
% end    
% A_constraint = [1; -1];
% b_constraint = [u_max; -u_min];
% 
% [u, ~, exitflag, ~] = quadprog(1, 0, A_constraint, b_constraint);
