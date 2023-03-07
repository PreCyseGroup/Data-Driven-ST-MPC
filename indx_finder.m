function index = indx_finder(x_curr,P)


% Written:      27-Feb-2023
% Last update:
% Last revision:---
% This function computes set membership index for model-based ROSC sets to
% used for model-based ST-MPC control algorithm 
      
%------------- BEGIN CODE --------------
steps = size(P,2);

for i=1:steps
    if P{i}.contains(x_curr)== 1
        index = i;
        break
    else
        index = [];
    end
end

end

%------------- END CODE --------------