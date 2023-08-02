function controlcommand = commandcalculation(x_curr,set_to_reach,A,B,inputconstraint,W)

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
H_x=set_to_reach.A;
g_x=set_to_reach.b;

%constraint on U (vincolo_u)
H_u=inputconstraint.A;
g_u=inputconstraint.b;

%Model (A,B)
const1 =[H_x*(A*x_curr+B*u)<=g_x - W];
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
% x_next=A*x_curr+B*controlcommand+[0.0011;-0.0001];

end

%------------- END CODE --------------