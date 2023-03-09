
% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:
% Last revision:---

%------------- BEGIN CODE --------------

clc
clear all
close all

rand('seed',1);
load T
load T_data
load T_data_aug

%% Computing ST-MPC control signals and state evolution of the system (model-based & data-driven)

figure;
% set(gca, 'Position',[0.099 0.12 0.897 0.87])
index = 1;
projectedDims = {[1 2]};

for plotRun=1: length(projectedDims)
    index = index + 1;
    
    % plot RCI set
    handleX0 = plot(T{1}, 'Alpha', 0.5, 'color', 'green');
    
    % plot model-based ROSC sets
    hold on
    for iSet=2:6
        handleModel=  plot(T{iSet},'Alpha',0.1,'color','white','EdgeColor','k');
    end
    hold on
    
    % plot ROSC sets from data
    for iSet=2:16
        handleData = plot(T_data{iSet},projectedDims{plotRun},'r--','LineWidth',0.75);
    end
    
    %%%%
    xlabel('$x_1$','interpreter','latex','FontSize',20)
    ylabel('$x_2$','interpreter','latex','FontSize',20)
    warOrig = warning; warning('off','all');
    exportgraphics(gcf,'STMPC_controller.eps','BackgroundColor','none','ContentType','vector')
end
%
grid off

hold on
x_curr = [-2; 1.1]; % initial state
x1 = x_curr;
U = Polyhedron('lb',model.u.min,'ub',model.u.max);
W = zonotope(zeros(2,1), 0.005*eye(2,2));

index_data = [];  % set membership for data-driven ST-MPC
index_model = [];  % set membership for model-based ST-MPC
index_data(1) = set_index(x1,T_data, T_data{2}, N+1)
index_model(1) = indx_finder(x_curr, T)
u1 = [];
i = 0;

% plot initial state
handleInitial_state = plot(x_curr(1),x_curr(2),'go','MarkerFaceColor', 'g',...
    'MarkerEdgeColor','k','MarkerSize',4)
hold on
% pause(7)
data_traj_x1(1) = x1(1);
data_traj_x2(1) = x1(2);
model_traj_x1(1) = x1(1);
model_traj_x2(1) = x1(2);
sim_time = 41;

W = zonotope(zeros(2,1), 0.005*eye(2,2));
while i < sim_time
    W_k = randPoint(W,1,'standard');
    % plot initial state
    hold on
    u1 = one_step_ctrl(1, x1, T_data_aug, index_data(i+1));
    x1 = A*x1 + B*u1 + W_k;
    u_data(i+1) = u1;
    hand_data_traj = plot(x1(1), x1(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k','MarkerSize', 3)
    hold on
    data_traj_x1(i+2) = x1(1);
    data_traj_x2(i+2) = x1(2);
    
    if index_model(i+1) == 1
        j = 1;
        controlcommand = -K*x_curr;
        x_next=A*x_curr+B*controlcommand+W_k;
    else
        j = index_model(i+1) - 1;
        [controlcommand ,x_next] = commandcalculation(x_curr,T{j},A,B,U,W_k);
    end
    u_model(i+1) = controlcommand;
    x_curr = x_next;
    model_traj_x1(i+2) = x_next(1);
    model_traj_x2(i+2) = x_next(2);
    
    % plot state evolution
    hand_model_traj = plot(x_curr(1),x_curr(2),'bo', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b','MarkerSize', 3)
    hold on
    index_model(i+2) = indx_finder(x_curr,T);
    index_data(i+2) = set_index(x1,T_data,T_data{2},N);
    pause(0.1)
    i = i + 1;
 
end
legend([handleX0,handleModel,handleData,handleInitial_state,hand_model_traj,hand_data_traj],...
        'RCI set using the model','ROSC sets using the model','ROSC sets using the data',...
        '$x_0$','$x_k$ using ST-MPC','$x_k$ using D-ST-MPC','Location','northeast',...
        'EdgeColor',[0.7 0.7 0.7], 'interpreter','Latex','FontSize',9.5);
%% Plot set membership index and ST-MPC control commands
figure;
time=0:sim_time-1;

subplot(2, 1, 1)
plot(time, index_data(1:i)-1, 'ko-', 'MarkerSize', 3, 'MarkerFaceColor', 'k','LineWidth', 1)
hold on
plot(time, index_model(1:i)-1, 'bo--', 'MarkerSize', 3, 'MarkerFaceColor', 'b', 'LineWidth', 1)
ylabel('$j_k$','interpreter','latex','FontSize',20)
legend(['Data-driven ST-MPC'],['Model-based ST-MPC'],'FontSize',10)
ylim([-1 16]);
xlim([0 20]);
set(gca, 'XTick', []);
subplot(2, 1, 2);
plot(time, u_data, 'ko-', 'MarkerSize', 3, 'MarkerFaceColor', 'k', 'LineWidth', 1)
hold on
plot(time, u_model, 'bo--', 'MarkerSize', 3, 'MarkerFaceColor', 'b', 'LineWidth', 1)
hold on
yline(3, 'r--', 'LineWidth', 1)
hold on
yline(-3, 'r--', 'LineWidth', 1)
ylabel('$u_k$', 'interpreter', 'latex','FontSize', 20)
legend(['Data-driven ST-MPC'], ['Model-based ST-MPC'], ['Input constraints'], 'FontSize',10)
xlabel('$k$', 'interpreter', 'latex', 'FontSize', 20)
ylim([-4 4]);
xlim([0 20]);
%------------- END CODE --------------

%%
figure;
set(gca, 'Position',[0.091 0.12 0.897 0.87])
index=1;
projectedDims = {[1 2]};

for plotRun=1:length(projectedDims)
    
    index=index+1;
    % plot initial set
    handleX0 = plot(T{1},'Alpha',0.5,'color','green');
    %     alpha(0.2)
    %     hold on;
    %     handleT0 = plot(C_approx{1});
    hold on
    for iSet=2:6
        handleModel=  plot(T{iSet},'Alpha',0.001,'color','gray','EdgeColor','k');
        %         alpha(0.1)
    end
    hold on
    % plot reachable sets from data
    for iSet=2:16
        handleData = plot(T_data{iSet},projectedDims{plotRun},'r--','LineWidth',0.8);
    end
    hold on
    initial_state = plot(-2,1.1,'gs','MarkerFaceColor', 'g','MarkerEdgeColor','k','MarkerSize',7);
    hold on
    model_trajectory = plot(model_traj_x1,model_traj_x2,'bs-','MarkerFaceColor', 'b',...
        'MarkerEdgeColor','blue','MarkerSize',2.5,'LineWidth',1);
    data_trajectory = plot(data_traj_x1,data_traj_x2,'ks--','MarkerFaceColor', 'k',...
        'MarkerEdgeColor','black','MarkerSize',2.5,'LineWidth',1);
    
    xlabel('$x_1$','interpreter','latex','FontSize',20)
%     ylabel('$x_2(k)$','interpreter','latex','FontSize',13)
    
    label_h = ylabel('$x_2$','interpreter','latex','FontSize',20);
%     label_h.Position(1)-2; % change horizontal position of ylabel
    label_h.Position(1) = label_h.Position(1)+0.15; % change vertical position of ylabel
    
    warOrig = warning; warning('off','all');
    %     warOrig = warning; warning('off','all');
    legend([handleX0,handleModel,handleData,initial_state,model_trajectory,data_trajectory],...
        'RCI set using the model','ROSC sets using the model','ROSC sets using the data',...
        '$x_0$','$x_k$ using ST-MPC','$x_k$ using D-ST-MPC','Location','northeast',...
        'EdgeColor',[0.7 0.7 0.7], 'interpreter','Latex','FontSize',9.5);
    
    %         t = annotation('textbox', 'String','$\hat{\mathcal{C}}$','interpreter','Latex');
    %         t.EdgeColor = 'w'
    exportgraphics(gcf,'STMPC_controller.eps','BackgroundColor','none','ContentType','vector')
    
    grid off
    box off
end
ax=axes;
set(ax,'units','normalized','position',[0.15,0.2,0.25,0.25])
box(ax,'on')
plot(T{1},'Alpha',0.5,'color','green','parent',ax)
hold on 
for iSet=2:6
        handleModel=  plot(T{iSet},'Alpha',0.001,'color','gray','EdgeColor','k');
        %         alpha(0.1)
end
hold on 
for iSet=2:16
        handleData = plot(T_data{iSet},projectedDims{plotRun},'r--','LineWidth',0.8);
end
hold on
    model_trajectory = plot(model_traj_x1,model_traj_x2,'bs-','MarkerFaceColor', 'b',...
        'MarkerEdgeColor','blue','MarkerSize',2,'LineWidth',1);
    data_trajectory = plot(data_traj_x1,data_traj_x2,'ks--','MarkerFaceColor', 'k',...
        'MarkerEdgeColor','black','MarkerSize',2,'LineWidth',1);
set(ax,'xlim',[-0.15,0.15],'ylim',[-0.15,0.15])
box on
ax.XColor = 'k';
ax.YColor = 'k';
% set(gcf,'renderer','Painters')
print -depsc -tiff -r300 -painters ST_MPC_controllable_sets.eps

