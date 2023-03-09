function T0 = computeRPI(Acl,alpha,W)

% Written:      27-Feb-2023
% Last update:
% Last revision:---
% This function computes RCI set using model parameters based on the
% proposed method in 'Invariant approximations of the minimal robust
% positively invariant set' by Rokovic
      
%------------- BEGIN CODE --------------

Wa = alpha*W ; 

%%calculate the RPI region for polytopes, by fixing alpha, we find the 's'
%%that satisfies the equation (4) in the paper.
for s=1:100
    Ws =(Acl^s)*W ;
    tf = Wa.contains(Ws);
    if tf==1
    break
    end
end

Fs=W;
for i=1:s-1
      Fi = (Acl^i)*W;
      Fs = plus(Fs,Fi);
end

%%Refering to equation (5) of the paper, the RPI terminal region is:
al= 1/(1-alpha);
T0 = al *Fs;


end

%------------- END CODE --------------
