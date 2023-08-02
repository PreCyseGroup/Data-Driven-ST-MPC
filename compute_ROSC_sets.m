
% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:  13-May-2023
% Last revision: 01-August-2023

%------------- BEGIN CODE --------------
clc
clear all
close all
rand('seed',1);
% Discrete system x(k+1)=Ax(k)+Bu(k).

w = warning ('off','all');
rmpath('folderthatisnotonpath')
warning(w)

A = [0.7969 -0.2247;
    0.1798 0.9767];
B = [0.1271;0.0132];
C = [1,0];
D = 0;

model = LTISystem('A',A,'B',B);

% constraint on states
model.x.min = [-10;-10];
model.x.max = [10;10];
X = Polyhedron('lb',model.x.min,'ub',model.x.max);

% constraint on input
model.u.min = [-3];
model.u.max = [3];
mu = model.u.min;
mx = model.u.max;
U = Polyhedron('lb',model.u.min,'ub',model.u.max);

% Define noise zonotope 'W'
W = zonotope(zeros(2,1),0.005*eye(2));
W = W.mptPolytope.P;


%%  Design Terminal RCI region

% quadratic penalties
model.x.penalty = QuadFunction([1 0; 0 1]);
model.u.penalty = QuadFunction(model.nu);

% get the LQR feedback
K = model.LQRGain;

%Find the terminal region
Acl = A+(B*K);

%compute RCI region
alpha = 0.6;
T0 = computeRPI(Acl,alpha,W);

% Considering an over-approximation of the RCI set
X0 = zonotope(zeros(2,1),0.106*[0.999 -0.1;-1 -1]);

% Visualization of the RCI set and it's zonotopic over-approximation set
figure;

plot(X0,[1 2],'r--','LineWidth',1)
hold on
plot(T0,'Alpha',0.3,'color','g');
xlabel('$x_1$','interpreter','latex','FontSize',20)
ylabel('$x_2$','interpreter','latex','FontSize',20)
legend(['RCI set based on model'],['Zonotopic over-approximation of RCI '])
box off
%% compute all possible A and B

U = zonotope(0,3);
X = zonotope(zeros(2,1),10*eye(2));
%noise zontope W
W = zonotope(zeros(2,1),0.005*eye(2));

sys = ss(A,B,C,D);  % define system
V_AB = compute_AB(sys,X,U,W);

for i=1:size(V_AB,1)
    A_hat{i} = V_AB{i}(1:2,1:2);
    B_hat{i} = V_AB{i}(1:2,3);
end
%% computing model-based and data-driven ROSC sets

% define constraints and noise as zonotopes
X = zonotope(zeros(2,1),10*eye(2));
U = zonotope(0,3);
W = zonotope(zeros(2,1),0.005*eye(2));
%
gen = [-2 -2 3.7;0.5 1 -4.5;5.5 -0.1 1]; % generator vector for zonotopic inner-approximation
N = 20;  % number of steps
T_data{1} = X0;
T{1} = T0;

tic;
for step=1:N
    T{step+1} = inv(A)*((T{step} - W.mptPolytope.P) + (-B*U.mptPolytope.P));
end
delta_t_model_based = toc;

tic;
for step=1:N
    T_data{step+1} = compute_intersec(compute_presets_approx(T_data{step}.mptPolytope.P, ...
        W.mptPolytope.P, U.mptPolytope.P, X.mptPolytope.P, A_hat, B_hat));
    T_data{step+1} = poly_approx(T_data{step+1}, size(gen,2), gen);
    T_data_aug{step+1} = T_data{step+1};
    T_data{step+1} = project(T_data{step+1},[1 2]);
    step
end
delta_t_data_driven = toc;
%% Visualization of model-based and data-driven ROSC sets (model-based & data-drive)
figure;

handleX0 = plot(T{1}, 'Alpha', 0.8, 'color', 'green')
hold on
for i=2:6
    handleModel = plot(T{i}, 'Alpha', 0.01, 'color', 'white');
    hold on
end
hold on
for i=2:16
    handleData = plot(T_data{i}, [1 2], 'r--', 'LineWidth', 0.75)
    hold on
end
xlabel('$x_1$','interpreter','latex','FontSize',20)
ylabel('$x_2$','interpreter','latex','FontSize',20)

legend([handleX0, handleModel, handleData],...
    'RCI set using the model', 'ROSC sets using the model', 'ROSC sets using the data', 'Location','northeast',...
    'EdgeColor',[0.7 0.7 0.7]);

save T 
save T_data
save T_data_aug


%------------- END OF CODE --------------