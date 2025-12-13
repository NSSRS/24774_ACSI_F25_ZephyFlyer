%% detect_bad_traj_all.m
% Diagnostic tool to find corrupted or abnormal Crazyflie trajectory logs
clear; close all; clc;

% ============================
% Folder paths (relative to this script)
% ============================
folders = { ...
    'data/circle_pid', ...
    'data/circle_disturbance_25_pid', ...
    'data/circle_disturbance_50_pid' ...
};
labels = {'No Wind', '25% Disturbance', '50% Disturbance'};

% ============================
% Diagnostic thresholds
% (you can tweak these)
% ============================
VAR_THRESHOLD = 0.5;      % [m^2] variance above which run is flagged
PATH_THRESHOLD = 5.0;     % [m] path length above which run is flagged

% ============================
% Loop through each condition
% ============================
for i = 1:numel(folders)
    folder_path = folders{i};
    files = dir(fullfile(folder_path, '*.txt'));
    if isempty(files)
        warning('No .txt files found in %s', folder_path);
        continue;
    end

    fprintf('\n======================================================\n');
    fprintf('=== CONDITION: %s (%s) ===\n', labels{i}, folder_path);
    fprintf('======================================================\n\n');

    % Create figure for XY top-down trajectory visualization
    figure('Name', ['Diagnostics - ' labels{i}], 'NumberTitle', 'off');
    tiledlayout('flow');
    sgtitle(['XY Trajectories - ' labels{i}]);

    % Loop over every log file
    for f = 1:numel(files)
        file_path = fullfile(folder_path, files(f).name);

        % --- Read file (skip header)
        fid = fopen(file_path, 'r');
        firstLine = fgetl(fid);
        fclose(fid);
        if contains(firstLine, 'time') || contains(firstLine, '#')
            data = readmatrix(file_path, 'NumHeaderLines', 1);
        else
            data = readmatrix(file_path);
        end

        if size(data,2) < 4
            warning('Skipping %s (unexpected format)', files(f).name);
            continue;
        end

        % --- Extract XYZ position
        pos = data(:,2:4);
        pos = pos(~any(isnan(pos),2), :);
        if isempty(pos), continue; end

        % --- Compute statistics
        mean_xyz = mean(pos, 1);
        var_xyz  = var(pos, 0, 1);
        diffs = diff(pos);
        seg_len = sqrt(sum(diffs.^2, 2));
        total_path = sum(seg_len);

        % --- Check if run is potentially bad
        flagged = any(var_xyz > VAR_THRESHOLD) || total_path > PATH_THRESHOLD;

        % --- Console report
        if flagged
            fprintf('*** FLAGGED *** ');
        else
            fprintf('File %2d: ', f);
        end
        fprintf('%-35s\n', files(f).name);
        fprintf('  Mean [x y z] = [%.3f  %.3f  %.3f]\n', mean_xyz);
        fprintf('  Var  [x y z] = [%.5f  %.5f  %.5f]\n', var_xyz);
        fprintf('  Total path length = %.3f m\n\n', total_path);

        % --- Plot trajectory
        nexttile;
        plot(pos(:,1), pos(:,2), 'LineWidth', 1.2);
        hold on;
        plot(mean_xyz(1), mean_xyz(2), 'rx', 'MarkerSize', 8, 'LineWidth', 1.2);
        title(strrep(files(f).name, '_', '\_'), 'Interpreter', 'tex', 'FontSize', 8);
        xlabel('X [m]'); ylabel('Y [m]');
        axis equal; grid on;
        if flagged
            % visually mark flagged trajectories in red
            set(gca, 'Color', [1 0.9 0.9]);
        end
    end
end

fprintf('\nDiagnostics complete.\n');
fprintf('Flagging criteria:\n');
fprintf('  Variance > %.2f m^2  OR  Path length > %.2f m\n', VAR_THRESHOLD, PATH_THRESHOLD);
fprintf('Review flagged runs visually and delete their .txt files if needed.\n');
