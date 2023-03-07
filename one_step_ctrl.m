function command = one_step_ctrl(dim_u,x_curr,aug_set,index)

% Author:       Mehran Attar
% Written:      27-Feb-2023
% Last update:
% Last revision:---
% This function computes the ST-MPC control commands by leveraging the
% data-driven ROSC sets
      
%------------- BEGIN CODE --------------

u = sdpvar(dim_u,1);
const = aug_set{index}.mptPolytope.P.A * [x_curr;u] <= aug_set{index}.mptPolytope.P.b;
obj = norm(u,2);
opt = sdpsettings('verbose',1,'debug',1);
diagnostics=optimize(const,obj,opt);
command = value(u);

end

%------------- END CODE --------------