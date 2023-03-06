function index = set_index(x,Preset, target_set, steps)
% Author:       Mehran Attar
% Written:      27-Feb-2023
% Last update:
% Last revision:---
% This function computes the set membership index for data-driven ROSC sets
% to used for data-driven ST-MPC control algorithm 


%------------- BEGIN CODE --------------
    for i=2:steps
        if Preset{i}.mptPolytope.P.A * x <= Preset{i}.mptPolytope.P.b == 1
            index = i;
            break
        end
    end

%------------- END CODE --------------
end