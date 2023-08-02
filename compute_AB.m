function V_AB = compute_AB(sys,X0,U,W)



% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:
% Last revision:---

%------------- BEGIN CODE --------------
rand('seed',1);

A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

dim_x = size(A, 2);
initpoints = 1;
steps = 5;
totalsamples = initpoints*steps;
%% initial set and input

%Construct matrix zonotpe \mathcal{M}_w
index=1;
for i=1:size(W.generators,2)
    vec = W.Z(:,i+1);
    GW{index}= [vec,zeros(dim_x,totalsamples-1)];
    for j=1:totalsamples-1
        GW{j+index}=  [GW{index+j-1}(:,2:end) GW{index+j-1}(:,1)];
    end
    index = j+index+1;
end
Wmatzono= matZonotope(zeros(dim_x,totalsamples),GW);

% randomly choose constant inputs for each step / sampling time
for i=1:totalsamples
    u(i) = randPoint(U);
end

%simulate the system to get the data
x0 = X0.center;
x(:,1) = x0;
index=1;
for j=1:dim_x:initpoints*dim_x
    x(j:j+dim_x-1,1) = randPoint(X0);
    for i=1:steps
        utraj(j,i) = u(index);
        x(j:j+dim_x-1,i+1) = A*x(j:j+dim_x-1,i) + B*u(index) + randPoint(W);
        index=index+1;
    end
end

% concatenate the data trajectories
index_0 =1;
index_1 =1;
for j=1:dim_x:initpoints*dim_x
    for i=2:steps+1
        x_meas_vec_1(:,index_1) = x(j:j+dim_x-1,i);
        index_1 = index_1 +1;
    end
    for i=1:steps
        u_mean_vec_0(:,index_0) = utraj(j,i);
        x_meas_vec_0(:,index_0) = x(j:j+dim_x-1,i);
        index_0 = index_0 +1;
    end
end

% X_+ is X_1T
% X_- is X_0T
U_full = u_mean_vec_0(:,1:totalsamples); %same as u
X_0T = x_meas_vec_0(:,1:totalsamples);
X_1T = x_meas_vec_1(:,1:totalsamples);

X1W_cen =  X_1T - Wmatzono.center;
X1W = matZonotope(X1W_cen,Wmatzono.generator);
% set of A and B
AB = X1W * pinv([X_0T;U_full]);

%%

matrixCenter = [AB.center;zeros(1,size(A,2)+size(B,2))];

for i=1:AB.gens
  G{i}=[AB.generator{i};zeros(1,size(A,2)+size(B,2))];  
end

% instantiate matrix zonotope
M_zono = matZonotope(matrixCenter, G);

% obtain result of all vertices 
V_AB = vertices(M_zono);



end

