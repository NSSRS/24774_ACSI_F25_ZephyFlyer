%% Crazyflie Circular Trajectory Analysis
% Reads all .txt logs in data folders, plots individual + mean trajectories,
% interactive checkbox GUI, and compares simulation vs real datasets.
clear; close all; clc;

% ============================
% Toggle simulation inclusion
% ============================
include_sim = true;  % set false to skip simulation datasets

% ============================
% Folder paths (relative)
% ============================
folders = { ...
    'data/circle_pid', ...
    'data/circle_pid_10s', ...
    'data/circle_disturbance_25_pid', ...
    'data/circle_disturbance_50_pid', ...
    'data/circle_sim_disturbance_5_pid', ...
    'data/circle_sim_pid' ...
};
labels = {'No Wind', 'No Wind (10s)', '25% Disturbance', '50% Disturbance', ...
          'Simulation (5 m/s Disturbance)', 'Simulation (No Wind)'};

if ~include_sim
    folders = folders(1:4);
    labels  = labels(1:4);
end

colors = lines(numel(folders));
mean_traj_all = cell(numel(folders),1);
t_all = cell(numel(folders),1);
isSim = contains(folders, 'sim');

% ============================
% Loop through each dataset
% ============================
for i = 1:numel(folders)
    folder_path = folders{i};
    files = dir(fullfile(folder_path, '*.txt'));
    all_traj = {};
    
    % --- Plot individual trajectories ---
    figure('Name', labels{i}, 'NumberTitle', 'off'); hold on;
    title(['Trajectories - ' labels{i}]);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    grid on; axis equal; view(3);
    
    for f = 1:numel(files)
        file_path = fullfile(folder_path, files(f).name);
        fid = fopen(file_path, 'r');
        firstLine = fgetl(fid); fclose(fid);
        if contains(firstLine, 'time') || contains(firstLine, '#')
            data = readmatrix(file_path, 'NumHeaderLines', 1);
        else
            data = readmatrix(file_path);
        end
        if size(data,2) < 7, continue; end
        
        pos_att = data(:,2:7); % [x y z roll pitch yaw]
        pos_att = pos_att(~any(isnan(pos_att),2), :);
        all_traj{end+1} = pos_att;
        
        % dashed for sim, solid for real
        if isSim(i), ls = '--'; else, ls = '-'; end
        plot3(pos_att(:,1), pos_att(:,2), pos_att(:,3), ...
            'Color',[0.7 0.7 0.7],'LineStyle',ls);
    end
    
    if isempty(all_traj)
        warning('No valid data in %s', folders{i});
        continue;
    end
    
    % --- Compute mean trajectory ---
    min_len = min(cellfun(@(x) size(x,1), all_traj));
    X = zeros(min_len,numel(all_traj));
    Y = X; Z = X; Roll = X; Pitch = X; Yaw = X;
    for k = 1:numel(all_traj)
        traj = all_traj{k};
        X(:,k)=traj(1:min_len,1); Y(:,k)=traj(1:min_len,2); Z(:,k)=traj(1:min_len,3);
        Roll(:,k)=traj(1:min_len,4); Pitch(:,k)=traj(1:min_len,5); Yaw(:,k)=traj(1:min_len,6);
    end
    mean_traj = [mean(X,2),mean(Y,2),mean(Z,2)];
    mean_traj_all{i} = mean_traj;
    
    % --- Store time vector ---
    file_path = fullfile(folder_path, files(1).name);
    fid = fopen(file_path,'r'); firstLine=fgetl(fid); fclose(fid);
    if contains(firstLine,'time') || contains(firstLine,'#')
        data_tmp = readmatrix(file_path,'NumHeaderLines',1);
    else
        data_tmp = readmatrix(file_path);
    end
    t_all{i} = data_tmp(1:min_len,1) - data_tmp(1,1);
    
    % --- Overlay mean trajectory ---
    plot3(mean_traj(:,1),mean_traj(:,2),mean_traj(:,3),...
        'Color',colors(i,:),'LineWidth',2);
    
    % --- Mean-only figure ---
    figure('Name',[labels{i} ' - Mean Trajectory'],'NumberTitle','off'); hold on;
    plot3(mean_traj(:,1),mean_traj(:,2),mean_traj(:,3),...
        'Color',colors(i,:),'LineWidth',2);
    title(['Mean Trajectory - ' labels{i}]);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    grid on; axis equal; view(3);
    
    % --- Print variance/std stats ---
    var_vals=[mean(var(X,0,2)),mean(var(Y,0,2)),mean(var(Z,0,2)),...
              mean(var(Roll,0,2)),mean(var(Pitch,0,2)),mean(var(Yaw,0,2))];
    std_vals=sqrt(var_vals);
    fprintf('=== %s ===\n',labels{i});
    fprintf('Variance [X Y Z Roll Pitch Yaw]: [%.5f %.5f %.5f %.5f %.5f %.5f]\n',var_vals);
    fprintf('Std Dev  [X Y Z Roll Pitch Yaw]: [%.4f %.4f %.4f %.4f %.4f %.4f]\n\n',std_vals);
end

disp('✅ All datasets processed.');

% ============================
% Combined mean trajectory comparison
% ============================
figure('Name','All Mean Trajectories','NumberTitle','off');
hold on; grid on; axis equal; view(3);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Comparison of Mean Trajectories');
for i=1:numel(mean_traj_all)
    if isempty(mean_traj_all{i}), continue; end
    traj=mean_traj_all{i};
    if isSim(i), ls='--'; else, ls='-'; end
    plot3(traj(:,1),traj(:,2),traj(:,3),...
        'Color',colors(i,:),'LineWidth',2,'LineStyle',ls,...
        'DisplayName',labels{i});
end
legend show;

% ============================
% Checkbox GUI for selection
% ============================
uiFig=uifigure('Name','Select Datasets to Plot',...
    'Position',[200 200 280 50+30*numel(labels)]);
cbGroup=[];
for i=1:numel(labels)
    cb=uicheckbox(uiFig,'Text',labels{i},...
        'Position',[20,30*(numel(labels)-i+1),240,25],...
        'Value',false,...
        'ValueChangedFcn',@(src,event)updateCombinedPlot(uiFig,mean_traj_all,labels,colors,isSim));
    cbGroup=[cbGroup;cb]; %#ok<AGROW>
end
setappdata(uiFig,'cbGroup',cbGroup);
combinedFig=figure('Name','Selected Mean Trajectories','NumberTitle','off');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Combined Mean Trajectories (Selected)');
grid on; axis equal; hold on; view(3);

% ============================
% Callback function
% ============================
function updateCombinedPlot(uiFig,mean_traj_all,labels,colors,isSim)
cbGroup=getappdata(uiFig,'cbGroup');
fig=findobj('Type','figure','Name','Selected Mean Trajectories');
if isempty(fig)
    fig=figure('Name','Selected Mean Trajectories','NumberTitle','off');
end
figure(fig); clf(fig);
hold on; grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Combined Mean Trajectories (Selected)');
view(3); rotate3d on;
selected=false;
for i=1:numel(cbGroup)
    if cbGroup(i).Value && ~isempty(mean_traj_all{i})
        traj=mean_traj_all{i};
        if isSim(i), ls='--'; else, ls='-'; end
        plot3(traj(:,1),traj(:,2),traj(:,3),...
            'Color',colors(i,:),'LineWidth',2,'LineStyle',ls,...
            'DisplayName',labels{i});
        selected=true;
    end
end
legend show;
if ~selected
    text(0,0,0,'No datasets selected','HorizontalAlignment','center');
end
drawnow;
end

% ============================
% Compare Simulation vs Real
% ============================
idx_real_nominal=1;   % circle_pid
idx_sim_nominal=6;    % circle_sim_pid
idx_real_25pct=3;     % circle_disturbance_25_pid
idx_sim_5ms=5;        % circle_sim_disturbance_5_pid

pair_labels={'No Disturbance: Real vs Sim','25% vs 5 m/s Disturbance: Real vs Sim'};
pair_idx=[idx_real_nominal,idx_sim_nominal; idx_real_25pct,idx_sim_5ms];

for p=1:size(pair_idx,1)
    i_real=pair_idx(p,1); i_sim=pair_idx(p,2);
    min_len=min(size(mean_traj_all{i_real},1),size(mean_traj_all{i_sim},1));
    real_traj=mean_traj_all{i_real}(1:min_len,:);
    sim_traj=mean_traj_all{i_sim}(1:min_len,:);
    t=t_all{i_real}(1:min_len);
    
    diff_traj=sim_traj-real_traj;
    diff_dist=sqrt(sum(diff_traj.^2,2));
    
    mean_diff=mean(diff_traj,1);
    rms_diff=sqrt(mean(diff_traj.^2,1));
    rms_total=sqrt(mean(diff_dist.^2));
    fprintf('=== %s ===\n',pair_labels{p});
    fprintf('Mean Δ [X Y Z]: [%.4f %.4f %.4f] m\n',mean_diff);
    fprintf('RMS  Δ [X Y Z]: [%.4f %.4f %.4f] m\n',rms_diff);
    fprintf('Overall RMS ΔPos: %.4f m\n\n',rms_total);
    
    figure('Name',['Sim vs Real - ' pair_labels{p}],'NumberTitle','off'); hold on;
    plot(t,diff_traj(:,1),'r','LineWidth',1.5,'DisplayName','ΔX (Sim–Real)');
    plot(t,diff_traj(:,2),'g','LineWidth',1.5,'DisplayName','ΔY (Sim–Real)');
    plot(t,diff_traj(:,3),'b','LineWidth',1.5,'DisplayName','ΔZ (Sim–Real)');
    %plot(t,diff_dist,'--','Color',[0.4 0.4 0.4],'LineWidth',1.2,'DisplayName','‖Δ‖ (3D)');
    xlabel('Time [s]');
    ylabel('Difference [m]');
    title(['Simulation – Real Trajectory Error (' pair_labels{p} ')']);
    legend('show','Location','best'); grid on; xlim([t(1) t(end)]);
end
