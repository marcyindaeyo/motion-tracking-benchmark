%% Precision report with task-specific analysis (DTW or endpoint-based)
clear; clc;
close all

filename = 'aligned_task2_L.mat'; 

%%
clear; clc;
close all

filename = 'aligned_task2_R.mat';

%%
clear; clc;
close all

filename = 'aligned_task8_L.mat'; 

%%
clear; clc;
close all

filename = 'aligned_task8_R.mat'; 

%%

load(filename);  

if exist('result_l', 'var')
    result = result_l;
elseif exist('result_r', 'var')
    result = result_r;
else
    error('No result_l or result_r found');
end

time = result.time;
cam  = result.cam;
br   = result.br;
vive = result.vive;

% Show trajectory to select segments
figure;
titles = {'X', 'Y', 'Z'};
for i = 1:3
    subplot(3,1,i); hold on;
    plot(time, cam(:,i), 'b');
    plot(time, br(:,i), 'g');
    plot(time, vive(:,i), 'r');
    ylabel([titles{i} ' (m)']);
    if i == 1
        legend('CAM', 'BR', 'VIVE');
    end
    if i == 3
        xlabel('Time (s)');
    end
    title(['Trajectory - ' titles{i} ' axis']);
    grid on;
end
sgtitle('Click 6 points: start & end of 3 motion segments (on any subplot)');
[xs, ~] = ginput(6);  % 3 segments, each with 2 clicks
segment_times = reshape(xs, 2, [])';  % [start, end]

[~, name, ~] = fileparts(filename);
parts = split(name, '_');
task_id = parts{2};          
hand = upper(parts{3});      

% Segment trials
get_segments = @(data) cellfun(@(t) ...
    data(find(time >= t(1), 1):find(time >= t(2), 1), :), ...
    num2cell(segment_times, 2), 'UniformOutput', false);

cam_segments = get_segments(cam);
br_segments  = get_segments(br);

if strcmp(task_id, 'task8')  % Task 8: DTW + resample + full trajectory analysis
    align_all = @(segments) cellfun( ...
        @(seg) dtw_align_segment(segments{1}, seg), ...
        segments, 'UniformOutput', false);

    resample_all = @(segments) cellfun(@(seg) ...
        safe_interp_resample(seg, 100), segments, 'UniformOutput', false);


    aligned_cam = resample_all(align_all(cam_segments));
    aligned_br  = resample_all(align_all(br_segments));

    compute_repeatability = @(segments) mean(std(cat(3, segments{:}), 0, 3));
    std_cam = compute_repeatability(aligned_cam);
    std_br  = compute_repeatability(aligned_br);

    % Visualization
    labels = {'CAM', 'BR'};
    data_segments = {aligned_cam, aligned_br};
    colors = lines(3); 
    trial_labels = {'Trial 1', 'Trial 2', 'Trial 3'};
    figure;
    for d = 1:2 
        subplot(1,2,d); hold on;
        for i = 1:length(data_segments{d})
            traj = data_segments{d}{i};
            plot3(traj(:,1), traj(:,2), traj(:,3), ...
                'LineWidth', 1.5, 'Color', colors(i,:), 'DisplayName', trial_labels{i});
        end
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title([labels{d} ' - 3D Trajectories (DTW Aligned)']);
        legend('show'); axis equal; grid on; view(3);
    end
    sgtitle('Repeatability Visualization with DTW Alignment');


else  % Task 2: Endpoint + trajectory visualization

    % Define peak as the max absolute X position
    get_peak_point = @(traj) traj(find(abs(traj(:,1)) == max(abs(traj(:,1))), 1), :);
    get_start_point = @(traj) traj(1, :);
    get_end_point   = @(traj) traj(end, :);

    peak_cam = cell2mat(cellfun(get_peak_point, cam_segments, 'UniformOutput', false));
    peak_br  = cell2mat(cellfun(get_peak_point, br_segments,  'UniformOutput', false));
    start_cam = cell2mat(cellfun(get_start_point, cam_segments, 'UniformOutput', false));
    start_br  = cell2mat(cellfun(get_start_point, br_segments,  'UniformOutput', false));
    end_cam = cell2mat(cellfun(get_end_point, cam_segments, 'UniformOutput', false));
    end_br  = cell2mat(cellfun(get_end_point, br_segments,  'UniformOutput', false));

    % Compute std across trials of peak positions
    std_cam = std(peak_cam);
    std_br  = std(peak_br);

    % Visualization: 3D Trajectory + Peak + Start/End
    figure;
    labels = {'CAM', 'BR'};
    data_segments = {cam_segments, br_segments};
    peak_points = {peak_cam, peak_br};
    start_points = {start_cam, start_br};
    end_points = {end_cam, end_br};
    colors = lines(3);
    trial_labels = {'Trial 1', 'Trial 2', 'Trial 3'};

    for d = 1:2 
        subplot(1,2,d); hold on;
        h_traj = gobjects(3,1);  % 轨迹句柄
        for i = 1:3
            traj = data_segments{d}{i};
    
            % Plot trajectory, save handle for legend
            h_traj(i) = plot3(traj(:,1), traj(:,2), traj(:,3), ...
                'Color', colors(i,:), 'LineWidth', 1.5);
    
            % Plot peak point (no legend yet)
            if i == 1
                h_peak = scatter3(peak_points{d}(i,1), peak_points{d}(i,2), peak_points{d}(i,3), ...
                    60, colors(i,:), 'filled', 'MarkerEdgeColor', 'k');
                h_rest = scatter3(start_points{d}(i,1), start_points{d}(i,2), start_points{d}(i,3), ...
                    40, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
                scatter3(end_points{d}(i,1), end_points{d}(i,2), end_points{d}(i,3), ...
                    40, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
            else
                scatter3(peak_points{d}(i,1), peak_points{d}(i,2), peak_points{d}(i,3), ...
                    60, colors(i,:), 'filled', 'MarkerEdgeColor', 'k');
                scatter3(start_points{d}(i,1), start_points{d}(i,2), start_points{d}(i,3), ...
                    40, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
                scatter3(end_points{d}(i,1), end_points{d}(i,2), end_points{d}(i,3), ...
                    40, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.4);
            end
        end
    
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title([labels{d} ' - Task 2 Trajectories & Key Points']);
        legend([h_traj; h_peak; h_rest], ...
            {'Trial 1', 'Trial 2', 'Trial 3', 'Peak', 'Rest Point'}, 'Location', 'best');
        grid on; axis equal; view(3);
    end
    
    sgtitle('Task 2 - Repeatability: Peak + Rest Point Visualization');



end


% Compute 3D std
std_cam_3d = norm(std_cam) * 1000;
std_br_3d  = norm(std_br)  * 1000;

% Save to CSV
new_row = table({task_id}, {hand}, ...
    std_cam(1)*1000, std_cam(2)*1000, std_cam(3)*1000, std_cam_3d, ...
    std_br(1)*1000,  std_br(2)*1000,  std_br(3)*1000,  std_br_3d, ...
    'VariableNames', {'Task', 'Hand', ...
    'CAM_X_mm', 'CAM_Y_mm', 'CAM_Z_mm', 'CAM_3D_STD_mm', ...
    'BR_X_mm',  'BR_Y_mm',  'BR_Z_mm',  'BR_3D_STD_mm'});

csv_file = 'precision.csv';
if isfile(csv_file)
    old = readtable(csv_file, 'VariableNamingRule', 'preserve');
    new = [old; new_row];
else
    new = new_row;
end

writetable(new, csv_file);
fprintf('Repeatability results saved to %s\n', csv_file);



