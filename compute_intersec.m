function preset_final = compute_intersec(presets)

% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:
% Last revision:---
% This function computes the intersection of sets
      
%------------- BEGIN CODE --------------

preset_final = presets{1};

for i=1:size(presets,2)
   preset_final = preset_final & presets{i};  
end
end

%------------- END CODE --------------