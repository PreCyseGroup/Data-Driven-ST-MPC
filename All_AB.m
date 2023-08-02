% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:
% Last revision:---

%------------- BEGIN CODE --------------
clc
clear all
close all
%% system dynamics
A=[0.7969 -0.2247;
    0.1798 0.9767];
B=[0.1271;0.0132];
C = [1,0];
D = 0;
rand('seed',1);
% define continuous time system
% sys_c = ss(A,B,C,D);
% % convert to discrete system
% samplingtime = 0.01;
% sys_d = c2d(sys_c,samplingtime);
dim_x = size(A, 2);
initpoints =1;
%Number of time steps
steps = 4;
totalsamples = initpoints*steps;
%% initial set and input
% X0 = zonotope(zeros(dim_x,1),diag(ones(dim_x,1)));
% X0 = zonotope(zeros(2,1),0.106*[0.999 -0.1;-1 -1]);
U = zonotope(0,3);
X0 = zonotope(zeros(2,1),10*eye(2));
%noise zontope W
W = zonotope(zeros(dim_x,1),0.005*eye(dim_x));

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

matrixCenter = [AB.center;zeros(1,3)];

for i=1:AB.gens
  G{i}=[AB.generator{i};zeros(1,3)];  
end

% instantiate matrix zonotope
M_zono = matZonotope(matrixCenter, G);

% obtain result of all vertices-------------------------
V_AB = vertices(M_zono);

save V_AB
