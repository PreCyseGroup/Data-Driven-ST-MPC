function out_approx = poly_approx(P,num_gen,generator_matrix)
% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:
% Last revision:---
% This function computes a zonotopic inner approximation of a polyhedral
% based on the proposed method in 'Scalable zonotopic under-approximation
% of backward reachable sets for uncertain linear systems By Yang, Liren
% and Ozay, Necmiye. 
      
%------------- BEGIN CODE --------------

dim = P.Dim;
alpha = sdpvar(num_gen, 1);
center = sdpvar(dim, 1);
weight = zeros(num_gen, 1);

for i = 1:num_gen
    weight(i, :) = norm(generator_matrix(:, i), 2);
end

constraints = [P.A * center + abs(P.A * generator_matrix) * alpha <= P.b];
opt = sdpsettings('verbose',0);
diagnostics=optimize(constraints,sum(-weight .* log(alpha)),opt);
center = value(center);
generator = zeros (dim, num_gen);

for i = 1:num_gen
    generator(:, i) = generator_matrix(:, i) * value(alpha(i));
end
out_approx = zonotope(center,generator);
end

%------------- END CODE --------------