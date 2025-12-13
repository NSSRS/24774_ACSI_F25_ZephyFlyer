%% Crazyflie ESO Trajectory Analysis (Simulation + Hardware, Full GUI Version)
% Reads .txt logs from:
%   - Simulation logs (PID, PID+wind, PID+wind+ESO)
%   - Hardware logs (PID, PID+ESO, PID+ESO+wind)
%
% Handles:
%   - Old logs (PID only)
%   - Webots ESO logs (pos, eso_pos, eso_att, disturbances)
%   - Hardware logs in same structure
%
clear; close all; clc;
delete(findall(groot,'Type','Figure'));

%% ============================================================
% Folder and Dataset Setup
% ============================================================
folders = {
    'data/sim/circle_pid',              
    'data/sim/circle_pid_wind',        
    'data/sim/circle_pid_wind_eso',    
    'data/hardware/circle_pid',        
    'data/hardware/circle_pid_eso',      
    'data/hardware/circle_pid_eso_wind25',
    'data/hardware/circle_pid_wind25',
    'data/hardware/hover_pid_eso_wind25'

};

labels = {
    'Sim (PID)', ...
    'Sim (PID + Wind)', ...
    'Sim (PID + Wind + ESO)', ...
    'HW (PID)', ...
    'HW (PID + ESO)', ...
    'HW (PID + ESO, Wind25)', ...
    'HW (PID + Wind25)', ...
    'HW (Hover PID + ESO + Wind25)'

};

colors = lines(numel(folders));
mean_traj_all = cell(numel(folders),1);
t_all = cell(numel(folders),1);

isSim = contains(folders,'sim');

%% ============================================================
% Loop through each dataset directory
% ============================================================
for i = 1:numel(folders)

    folder_path = folders{i};
    files = dir(fullfile(folder_path,'*.txt'));
    all_traj = {};

    fprintf("\n=== Processing %s ===\n", labels{i});

    fig_i = figure('Name',labels{i},'NumberTitle','off'); hold on;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(['Trajectories - ' labels{i}]);
    grid on; axis equal; view(3);

    for f = 1:numel(files)
        file_path = fullfile(folder_path, files(f).name);
        raw = readmatrix(file_path);
        nCols = size(raw,2);

        % ================================================================
        % Detect LOG FORMAT (Hardware, Old Simulation, or Webots ESO)
        % ================================================================
        if nCols >= 15
            % New ESO-enabled log format
            fprintf("Detected ESO log: %s\n", files(f).name);

            pos = raw(:,2:4);          % xyz
            eso_pos = raw(:,5:7);      % eso xyz
            eso_att = raw(:,8:10);     % roll pitch yaw
            eso_dist = raw(:,11:13);   % Fx Fy Fz

            pos_att = [pos, eso_att];

        else
            % Old format
            fprintf("Detected old standard log: %s\n", files(f).name);
            pos_att = raw(:,[2,3,4,10,11,12]);
        end

        pos_att = pos_att(~any(isnan(pos_att),2),:);
        all_traj{end+1} = pos_att;

        ls = '--'; if ~isSim(i), ls = '-'; end

        % Plot trajectory
        plot3(pos_att(:,1),pos_att(:,2),pos_att(:,3), ...
              'Color',[0.7 0.7 0.7],'LineStyle',ls);

        % ===============================================================
        % Add Force Vectors (ONLY valid for new ESO logs)
        % ===============================================================
        if nCols >= 17
            % Force columns in new log format
            dist_w = raw(:,15:17);   % world-frame ESO disturbance forces
            xyz    = raw(:,2:4);     % plot forces at CF position

            % Downsample to avoid clutter
            step = max(10, floor(size(raw,1)/200));
            xyz_ds  = xyz(1:step:end,:);
            dist_ds = dist_w(1:step:end,:);

            % Scale forces for visibility
            scale = 0.05;
            dist_ds = dist_ds * scale;

            quiver3( ...
                xyz_ds(:,1), xyz_ds(:,2), xyz_ds(:,3), ...
                dist_ds(:,1), dist_ds(:,2), dist_ds(:,3), ...
                0, 'Color', [1 0 0], 'LineWidth', 1.1);
        end
    end

    if isempty(all_traj)
        warning("No valid trajectories in %s", folder_path);
        continue;
    end

    %% Compute Mean Trajectory
    min_len = min(cellfun(@(x) size(x,1), all_traj));
    X = zeros(min_len, numel(all_traj));
    Y = X; Z = X;

    for k = 1:numel(all_traj)
        X(:,k) = all_traj{k}(1:min_len,1);
        Y(:,k) = all_traj{k}(1:min_len,2);
        Z(:,k) = all_traj{k}(1:min_len,3);
    end

    mean_traj = [mean(X,2), mean(Y,2), mean(Z,2)];
    mean_traj_all{i} = mean_traj;

    % Time for dataset
    tmp = readmatrix(fullfile(folder_path,files(1).name));
    t_all{i} = tmp(1:min_len,1) - tmp(1,1);

    % Plot mean trajectory
    plot3(mean_traj(:,1),mean_traj(:,2),mean_traj(:,3), ...
        'Color',colors(i,:),'LineWidth',2);
end

disp("All datasets processed.");

%% ============================================================
% Master Comparison Plot (All Means Together)
% ============================================================
figure('Name','All Mean Trajectories','NumberTitle','off'); hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Comparison of Mean Trajectories');
grid on; axis equal; view(3);

for i = 1:numel(mean_traj_all)
    if isempty(mean_traj_all{i}), continue; end

    ls = '--'; if ~isSim(i), ls = '-'; end

    plot3(mean_traj_all{i}(:,1),mean_traj_all{i}(:,2),mean_traj_all{i}(:,3), ...
        'Color',colors(i,:),'LineWidth',2,'LineStyle',ls, ...
        'DisplayName',labels{i});
end

legend show;

%% ============================================================
% GUI Checkboxes for Selecting Mean Trajectories
% ============================================================
uiFig = uifigure('Name','Select Datasets',...
    'Position',[200 200 300 60+30*numel(labels)]);

cbGroup = [];
for i = 1:numel(labels)
    cb = uicheckbox(uiFig,'Text',labels{i}, ...
        'Position',[20,30*(numel(labels)-i+1),260,25], ...
        'Value',false, ...
        'ValueChangedFcn', ...
            @(src,event)updateCombinedPlot(uiFig,mean_traj_all,labels,colors,isSim));
    cbGroup = [cbGroup; cb];
end

setappdata(uiFig,'cbGroup',cbGroup);

combinedFig = figure('Name','Selected Mean Trajectories','NumberTitle','off');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Combined Mean Trajectories (Selected)');
grid on; axis equal; hold on; view(3);

%% ============================================================
% Callback Function
% ============================================================
function updateCombinedPlot(uiFig, mean_traj_all, labels, colors, isSim)

cbGroup = getappdata(uiFig,'cbGroup');

fig = findobj('Type','figure','Name','Selected Mean Trajectories');
if isempty(fig)
    fig = figure('Name','Selected Mean Trajectories');
end

figure(fig); clf(fig);
hold on; grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Combined Mean Trajectories (Selected)');

selected = false;

for i = 1:numel(cbGroup)
    if cbGroup(i).Value && ~isempty(mean_traj_all{i})
        ls = '--'; if ~isSim(i), ls = '-'; end

        traj = mean_traj_all{i};
        plot3(traj(:,1),traj(:,2),traj(:,3), ...
            'Color',colors(i,:),'LineWidth',2,'LineStyle',ls, ...
            'DisplayName',labels{i});
        selected = true;
    end
end

if selected
    legend show;
else
    text(0,0,0,'No datasets selected',...
        'HorizontalAlignment','center');
end

drawnow;
end
