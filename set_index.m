function index = set_index(x,Preset)
% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:
% Last revision: 30-July-2023
% This function computes the set membership index for data-driven ROSC sets
% to used for data-driven ST-MPC control algorithm 


%------------- BEGIN CODE --------------
    for i=2:max(size(Preset))
        if Preset{i}.contains(x)== 1
%         if Preset{i}.mptPolytope.P.A * x <= Preset{i}.mptPolytope.P.b == 1
            index = i;
            break
        end
        index = i;
    end

%------------- END CODE --------------
end