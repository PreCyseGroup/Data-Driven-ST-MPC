function [x_next, controlcommand] = model_based_stmpc(x_curr,set_to_reach,A,B,inputconstraint,W,W_k)

% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:
% Last revision:---
% This function computes the ST-MPC control commands by leveraging the
% model-based ROSC sets 
      
%------------- BEGIN CODE --------------

%command input you want to find
u=sdpvar(size(B,2),1);
rng('default');
rng(1);
%set to reach (set_to_reach)
%poliedro  %Hx<=g
T_tilde = set_to_reach - W;

H_x=T_tilde.A;
g_x=T_tilde.b;

%constraint on U (vincolo_u)
H_u=inputconstraint.A;
g_u=inputconstraint.b;

%Model (A,B)
const1 =[H_x*(A*x_curr+B*u)<=g_x];
const2 = [H_u*u<=g_u];
const = const1 + const2;
%definition of the optimization problem
opt=sdpsettings('verbose',0);
obj =  u^2;
diagnostics=optimize(const,obj,opt);

% optimize(S,norm(u)+norm(A*x_curr+B*u))
%optimal value (command)
controlcommand=value(u);

%evolution in 1 step (x_next)
x_next=A*x_curr+B*controlcommand+W_k;

end

%------------- END CODE --------------