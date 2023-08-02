% Author:       Mehran Attar
% Written:      08-March-2023
% Last update:
% Last revision: 01-August-2023

% Demo for Data-Driven ST-MPC
clc
close all
clear all

run('compute_ROSC_sets.m');
run('compute_ST_MPC.m');
%%

close all

f = figure;
f.Position = [100 100 1400 600];
index=1;
projectedDims = {[1 2]};
frame = getframe;
for plotRun=1:length(projectedDims)
    index=index+1;
    % plot initial set
    subplot(2,2,[1 3])
    handleX0 = plot(T{1},'Alpha',0.5,'color','green');
    
    hold on
    for iSet=2:6
        handleModel = plot(T{iSet},'Alpha',0.01,'color','white','EdgeColor',[0.2784    0.2667    0.2667]);
      
    end
    hold on
   
    for iSet=2:16
        handleData = plot(T_data{iSet},projectedDims{plotRun},'r--','LineWidth',0.75);
    end
    
    xlabel('$x_1$','interpreter','latex','FontSize',20)
    ylabel('$x_2$','interpreter','latex','FontSize',20)
end

hold on
pause(6)
for i=2:21
   
    subplot(2,2,[1 3])
    grid off
    initial_state = plot(-2,1.1,'gs','MarkerFaceColor', 'g','MarkerEdgeColor','k','MarkerSize',7);
    hold on
    
    data_traject = plot(data_traj_x1(i),data_traj_x2(i),...
        'bo','MarkerFaceColor', 'b','MarkerEdgeColor','b','MarkerSize',3);
    hold on
    model_traject = plot(model_traj_x1(i),model_traj_x2(i),...
        'ko','MarkerFaceColor', 'k','MarkerEdgeColor','k','MarkerSize',3);
    %     subplot(2,2,[1 3]);
    
    
    title('State evolution')
    %     model_trajectory = plot(x_curr(1),x_curr(2),'ko','MarkerFaceColor', 'k','MarkerEdgeColor','k','MarkerSize',3);
    hold on
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ax=axes;
    set(ax,'units','normalized','position',[0.15,0.15,0.15,0.15])
    box(ax,'on')
    hold on
    handleX0 = plot(T{1},'Alpha',0.5,'color','green','parent',ax,'EdgeColor',[0.2784    0.2667    0.2667])
    hold on
    for iSet=2:6
        handleModel=  plot(T{iSet},'Alpha',0.001,'color','gray',...
            'EdgeColor',[0.2784    0.2667    0.2667]);
    
    end
    hold on
    for iSet=2:15
        handleData = plot(T_data{iSet},projectedDims{plotRun},'r--','LineWidth',0.8);
    end
    hold on
    initial_state = plot(-2,1.2,'gs','MarkerFaceColor', 'g','MarkerEdgeColor','k','MarkerSize',7);
    hold on
    data_trajectory = plot(data_traj_x1(i),data_traj_x2(i),...
        'bo','MarkerFaceColor', 'b','MarkerEdgeColor','b','MarkerSize',3);
    hold on
    model_trajectory = plot(model_traj_x1(i),model_traj_x2(i),...
        'ko','MarkerFaceColor', 'k','MarkerEdgeColor','k','MarkerSize',3);
    set(ax,'xlim',[-0.15,0.15],'ylim',[-0.15,0.15])
    box on
    ax.XColor = 'k';
    ax.YColor = 'k';
    hold on
    
    
    legend([handleX0,handleModel,handleData,initial_state,model_traject,data_traject],...
        'RCI set using the model','ROSC sets using the model','ROSC sets using the data',...
        '$x_0$','$x_k$ using ST-MPC','$x_k$ using D-ST-MPC','Location',[0.34 0.78 0.1 0.1],...
        'EdgeColor',[0.7 0.7 0.7], 'interpreter','Latex','FontSize',9.5);
    
    
    subplot(2,2,2)
    title('Control commmand')
    
    plot([i-2 i-1],[u_model(i-1) u_model(i)],'k-','LineWidth',1.5);
    hold on
    plot([i-2 i-1],[u_data(i-1) u_data(i)],'b-','LineWidth',1.5);
  
    hold on
  
    yline(-3,'r--','LineWidth',1);
    hold on
    yline(3,'r--','LineWidth',1);
    ylim([-5 5])
    xlim([0 20])
    xlabel('$k$','interpreter','latex','FontSize',20)
    ylabel('$u_k$','interpreter','latex','FontSize',20)
    legend(['ST-MPC'],['D-ST-MPC'],['Input constraints'])
    
    
    subplot(2,2,4)
    title('Set membership index')
    
    plot([i-2 i-1],[index_model(i-1)-1 index_model(i)-1],...
        'ko','MarkerFaceColor', 'k','MarkerEdgeColor','k','MarkerSize',3);
    hold on
    plot([i-2 i-1],[index_data(i-1)-1 index_data(i)-1],...
        'bo','MarkerFaceColor', 'b','MarkerEdgeColor','b','MarkerSize',3);
    
    xlabel('$k$','interpreter','latex','FontSize',20);
    ylabel('$j_k$','interpreter','latex','FontSize',20);
    legend(['ST-MPC'],['D-ST-MPC']);
    ylim([-1 20])
    xlim([0 20])
    pause(0.2)
    
end

